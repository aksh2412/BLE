/****************************************************************************
 *
 * This demo showcases how to simply advertise a ble packet. It can send adv data, be connected by client.
 * This will connect to ble client (moblie app) this will not require any pin.
 * BLE client can connect to LightBlue app from google play store. This is tested for android only.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define GATTS_TAG 						"BLE_ADV"

#define TEST_DEVICE_NAME            	"BLE_ADVERTISE"

#define GATT_SERVICE_NUM 				0

#define SVC_INST_ID             		0

/**
 * @enum gatt_services_database_t
 * @brief This enum contains bluetooth gatt service database
 */
typedef enum
{
	GATT_SERVICE, /*!< 0 - WiFi Gatt Services */
	GATT_SUM_DECLARE, /*!< 1 - WiFi Onboard Status Declare Gatt Services */
	GATT_SUM_VALUE, /*!< 2 - WiFi Onboard Status Value Gatt Services */
	GATT_FIRST_NUM_DECLARE, /*!< 3 - WiFi SSID Declare Gatt Services */
	GATT_FIRST_NUM_VALUE, /*!< 4 - WiFi SSID Value Gatt Services */
	GATT_SECOND_NUM_DECLARE, /*!< 5 - WiFi Password Declare Gatt Services */
	GATT_SECOND_NUM_VALUE, /*!< 6 - WiFi Password Value Gatt Services */
	END_OF_TABLE, /*!< 10 - END of Table */
} gatt_services_database;

/**
 * @struct gatts_proGATTS_TAG_inst_t
 * @brief This structure contains gatt service probal tag instance configurations
 */
typedef struct
{
	esp_gatts_cb_t gatts_cb; /*!< Gatt Services callback configuration. (ESP32) */
	uint16_t gatts_if; /*!< Gatt Services interface types */
	uint16_t service_id; /*!< Gatt Services app id */
} ble_gatt_profile;

static uint8_t adv_service_uuid128[32] =
{
/* LSB <--------------------------------------------------------------------------------> MSB */
//first uuid, 16bit, [12],[13] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, };

// The length of adv data must be less than 31 bytes
//adv data
static esp_ble_adv_data_t adv_data =
{ .set_scan_rsp = false, .include_name = true, .include_txpower =
false,
		.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
		.max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
		.appearance = 0x00, .manufacturer_len = 0,
		.p_manufacturer_data = NULL, //&test_manufacturer[0],
		.service_data_len = 0, .p_service_data = NULL, .service_uuid_len = sizeof(adv_service_uuid128),
		.p_service_uuid = adv_service_uuid128, .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };

static esp_ble_adv_params_t adv_params =
{ .adv_int_min = 0x20, .adv_int_max = 0x40, .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
//.peer_addr            =
//.peer_addr_type       =
		.channel_map = ADV_CHNL_ALL, .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, };

static const uint16_t SERVICE_UUID_CONSTANT = 0x00FF;
/** \brief Bluetooth WiFi Status Characteristic Constant */
static const uint16_t WIFI_STATUS_CHARACTERISTIC_UUID_CONSTANT = 0x1119;
/** \brief Bluetooth WiFi SSID Characteristic Constant */
static const uint16_t FIRST_NUM_CHARACTERISTIC_UUID_CONSTANT = 0x1120;
/** \brief Bluetooth WiFi PASSWORD Characteristic Constant */
static const uint16_t SECOND_NUM_CHARACTERISTIC_UUID_CONSTANT = 0x1121;

/** @brief This variable stores the gatt services pro bluetoooth instance */
ble_gatt_profile gatt_profile_config;
/** @brief This variable stores the gatt services attribute's database.(ESP32) */
esp_gatts_attr_db_t gatt_db[END_OF_TABLE];
/** @brief This variable stores the gatt services attribute handlers.(ESP32) */
uint16_t gatt_services_handles[END_OF_TABLE];


/** @brief This variable stores the gatt primary service UUID */
uint16_t gatt_primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
/** @brief This variable stores the gatt character declare UUID */
uint16_t gatt_char_declare_uuid = ESP_GATT_UUID_CHAR_DECLARE;


uint16_t num_first_declare_value = (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ);
/** @brief This variable stores the password declare value */
uint16_t num_second_declare_value = (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ);

/** @brief This variable stores the WiFi onboard status declare value */
uint16_t sum_declare_value = ESP_GATT_CHAR_PROP_BIT_READ;

/** @brief This variable stores first number to sum */
char gatt_num_first[10] = "";
/** @brief This variable stores second number to sum */
char gatt_num_second[10] = "";
/** @brief This variable stores sum of two numbers*/
char gatt_sum[10] = "";

/**
 * @brief       Declares services and characteristics and value declaration. Created attribute table.
 * @param  		gatts_if:  GATT interface type.
 * @return      ESP_OK if attribute table is created successfully.
 */
esp_err_t ble_create_gatt_database(esp_gatt_if_t gatts_if)
{
	memset(&gatt_db, 0, sizeof(gatt_db));

	// Service Declaration for first number, second number & sum Characteristic
	gatt_db[GATT_SERVICE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_SERVICE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_SERVICE].att_desc.uuid_p = (uint8_t*) &gatt_primary_service_uuid;
	gatt_db[GATT_SERVICE].att_desc.perm = ESP_GATT_PERM_READ;
	gatt_db[GATT_SERVICE].att_desc.max_length = sizeof(uint16_t);
	gatt_db[GATT_SERVICE].att_desc.length = sizeof(SERVICE_UUID_CONSTANT);
	gatt_db[GATT_SERVICE].att_desc.value = (uint8_t*) &SERVICE_UUID_CONSTANT;

	// Characteristic First Number Declaration
	gatt_db[GATT_FIRST_NUM_DECLARE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.uuid_p = (uint8_t*) &gatt_char_declare_uuid;
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.max_length = sizeof(uint16_t);
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.length = sizeof(FIRST_NUM_CHARACTERISTIC_UUID_CONSTANT);
	gatt_db[GATT_FIRST_NUM_DECLARE].att_desc.value = (uint8_t*) &num_first_declare_value;

	// Characteristic First Number value Declaration
	gatt_db[GATT_FIRST_NUM_VALUE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.uuid_p = (uint8_t*) &FIRST_NUM_CHARACTERISTIC_UUID_CONSTANT;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.max_length = 10;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.length = 10;
	gatt_db[GATT_FIRST_NUM_VALUE].att_desc.value = (uint8_t*) gatt_num_first;

	// Characteristic Second Number Declaration
	gatt_db[GATT_SECOND_NUM_DECLARE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.uuid_p = (uint8_t*) &gatt_char_declare_uuid;
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.max_length = sizeof(uint16_t);
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.length = sizeof(SECOND_NUM_CHARACTERISTIC_UUID_CONSTANT);
	gatt_db[GATT_SECOND_NUM_DECLARE].att_desc.value = (uint8_t*) &num_second_declare_value;

	// Characteristic Second Number value Declaration
	gatt_db[GATT_SECOND_NUM_VALUE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.uuid_p = (uint8_t*) &SECOND_NUM_CHARACTERISTIC_UUID_CONSTANT;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.max_length = 10;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.length = 10;
	gatt_db[GATT_SECOND_NUM_VALUE].att_desc.value = (uint8_t*) gatt_num_second;

	// Characteristic sum Declaration
	gatt_db[GATT_SUM_DECLARE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_SUM_DECLARE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_SUM_DECLARE].att_desc.uuid_p = (uint8_t*) &gatt_char_declare_uuid;
	gatt_db[GATT_SUM_DECLARE].att_desc.perm = ESP_GATT_PERM_READ;
	gatt_db[GATT_SUM_DECLARE].att_desc.max_length = sizeof(uint16_t);
	gatt_db[GATT_SUM_DECLARE].att_desc.length = sizeof(WIFI_STATUS_CHARACTERISTIC_UUID_CONSTANT);
	gatt_db[GATT_SUM_DECLARE].att_desc.value = (uint8_t*) &sum_declare_value;

	// Characteristic Sum value Declaration
	gatt_db[GATT_SUM_VALUE].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
	gatt_db[GATT_SUM_VALUE].att_desc.uuid_length = ESP_UUID_LEN_16;
	gatt_db[GATT_SUM_VALUE].att_desc.uuid_p = (uint8_t*) &WIFI_STATUS_CHARACTERISTIC_UUID_CONSTANT;
	gatt_db[GATT_SUM_VALUE].att_desc.perm = ESP_GATT_PERM_READ;
	gatt_db[GATT_SUM_VALUE].att_desc.max_length = 10;
	gatt_db[GATT_SUM_VALUE].att_desc.length = 10;
	gatt_db[GATT_SUM_VALUE].att_desc.value = (uint8_t*) &gatt_sum;

	return esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, END_OF_TABLE, SVC_INST_ID);
}

/**
 * @brief       A callback event when a gap request (esp_gap_ble_cb_event_t) is received.
 * @param  		event: event type esp_gap_ble_cb_event_t.
 *         		param: gap callback parameter esp_ble_gap_cb_param_t.
 * @return      none
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
		}
		else
		{
			ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTS_TAG,
				"update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status, param->update_conn_params.min_int, param->update_conn_params.max_int,
				param->update_conn_params.conn_int, param->update_conn_params.latency,
				param->update_conn_params.timeout);
		break;
	default:
		break;
	}
}

/**
 * @brief       A callback event when a gatt request (esp_gatts_cb_event_t) is received from  gatt client.
 * @param  		event: event type esp_gatts_cb_event_t
 *         		gatts_if:  GATT interface type.
 *         		param: received gatt callback parameter esp_ble_gatts_cb_param_t.
 * @return      none
 */
static void ble_gatts_sum_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
		esp_ble_gatts_cb_param_t *param)
{
	esp_err_t ret;

	switch (event)
	{
	case ESP_GATTS_REG_EVT:
		break;

	case ESP_GATTS_READ_EVT:
	{
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		if (param->read.handle == gatt_services_handles[GATT_SUM_VALUE])
		{
			int result = atoi(gatt_num_first) + atoi(gatt_num_second);

			memset(gatt_sum, 0x0, sizeof(gatt_sum));
			itoa(result, gatt_sum, 10);

			rsp.attr_value.handle = param->read.handle;
			rsp.attr_value.len = strlen(gatt_sum);

			memcpy(rsp.attr_value.value, gatt_sum, strlen(gatt_sum));

		}
		else if (param->read.handle == gatt_services_handles[GATT_FIRST_NUM_VALUE])
		{
			memcpy(rsp.attr_value.value, gatt_num_first, strlen(gatt_num_first));
		}
		else if (param->read.handle == gatt_services_handles[GATT_SECOND_NUM_VALUE])
		{
			memcpy(rsp.attr_value.value, gatt_num_second, strlen(gatt_num_second));
		}
		ret = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
		if (ret != ESP_OK)
			ESP_LOGE(GATTS_TAG, "%s : ESP_GATTS_READ_EVT : send response Failed : %d", __func__, ret);
	}
		break;

	case ESP_GATTS_WRITE_EVT:
	{
		ESP_LOGD(GATTS_TAG, "%s : ESP_GATTS_WRITE_EVT : handle %d", __func__, param->read.handle);

		if (param->write.handle == gatt_services_handles[GATT_FIRST_NUM_VALUE])
		{
			memset(gatt_num_first, 0x0, sizeof(gatt_num_first));
			memcpy(gatt_num_first, param->write.value, param->write.len);
			ESP_LOGI(GATTS_TAG, "%s : NUM first = %s", __func__, gatt_num_first);
		}
		else if (param->write.handle == gatt_services_handles[GATT_SECOND_NUM_VALUE])
		{
			memset(gatt_num_second, 0x0, sizeof(gatt_num_second));
			memcpy(gatt_num_second, param->write.value, param->write.len);
			ESP_LOGI(GATTS_TAG, "%s : NUM second = %s", __func__, gatt_num_second);
		}

		ret = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK,
		NULL);
		if (ret != ESP_OK)
			ESP_LOGE(GATTS_TAG, "%s : ESP_GATTS_WRITE_EVT : send response Failed : %d", __func__, ret);
	}
		break;

	default:
		break;
	}
}

/**
 * @brief       A callback event when a gatt request (esp_gatts_cb_event_t) is received from  gatt client.
 * @param  		event: event type esp_gatts_cb_event_t
 *         		gatts_if:  GATT interface type.
 *         		param: received gatt callback parameter esp_ble_gatts_cb_param_t.
 * @return      none
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT)
	{
		if (param->reg.status == ESP_GATT_OK)
		{
			gatt_profile_config.gatts_if = gatts_if;
			gatt_profile_config.service_id = param->reg.app_id;
		}

		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
		if (set_dev_name_ret)
		{
			ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
		}

		esp_err_t ret = ble_create_gatt_database(gatts_if);
		if (ret != ESP_OK)
			ESP_LOGE(GATTS_TAG, "%s : create attribute table failed : %d", __func__, ret);

		ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret)
		{
			ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
		}

		esp_ble_gap_start_advertising(&adv_params);
	}

	if (event == ESP_GATTS_CREAT_ATTR_TAB_EVT)
	{
		if (param->add_attr_tab.status != ESP_GATT_OK)
		{
			ESP_LOGD(GATTS_TAG, "%s : ESP_GATTS_CREAT_ATTR_TAB_EVT : create attribute table failed : %d", __func__,
					param->add_attr_tab.status);
		}
		else if (param->add_attr_tab.num_handle != END_OF_TABLE)
		{
			ESP_LOGE(GATTS_TAG, "%s : ESP_GATTS_CREAT_ATTR_TAB_EVT : create attribute table abnormally : %d", __func__,
					param->add_attr_tab.num_handle);
		}
		else
		{
			ESP_LOGD(GATTS_TAG, "%s : ESP_GATTS_CREAT_ATTR_TAB_EVT : create attribute table successful : %d", __func__,
					param->add_attr_tab.num_handle);
			memcpy(gatt_services_handles, param->add_attr_tab.handles, sizeof(gatt_services_handles));
			esp_ble_gatts_start_service(gatt_services_handles[GATT_SERVICE]);
		}
	}

	if (event == ESP_GATTS_READ_EVT || event == ESP_GATTS_WRITE_EVT)
	{
		gatt_profile_config.gatts_cb(event, gatts_if, param);
	}

	if (event == ESP_GATTS_DISCONNECT_EVT)
	{
		esp_ble_gap_start_advertising(&adv_params);
	}

}

/**
 * @brief       Main function that releases existing ble memory and initializes & enable ble controller.
 * 				It also initialize and enable bluedroid register callback for GATT and GAP.
 * 				Registers a GATT profile.
 * @param  		none
 * @return      none
 */
void app_main(void)
{
	esp_err_t ret;

	// Initialize NVS.
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	gatt_profile_config.gatts_cb = ble_gatts_sum_event_handler;
	gatt_profile_config.gatts_if = ESP_GATT_IF_NONE;

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
	;
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_init();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(GATT_SERVICE_NUM);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	return;
}
