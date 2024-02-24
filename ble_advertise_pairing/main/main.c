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

#define PROFILE_A_APP_ID 				0

static uint8_t adv_service_uuid128[32] =
{
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
		0xEE, 0x00, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
		0xFF, 0x00, 0x00, 0x00, };

// The length of adv data must be less than 31 bytes
//adv data
static esp_ble_adv_data_t adv_data =
{ .set_scan_rsp = false, .include_name = true, .include_txpower =
false, .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
		.max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
		.appearance = 0x00, .manufacturer_len = 0,
		.p_manufacturer_data = NULL, //&test_manufacturer[0],
		.service_data_len = 0, .p_service_data = NULL, .service_uuid_len =
				sizeof(adv_service_uuid128), .p_service_uuid =
				adv_service_uuid128, .flag = (ESP_BLE_ADV_FLAG_GEN_DISC
				| ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };

static esp_ble_adv_params_t adv_params =
{ .adv_int_min = 0x20, .adv_int_max = 0x40, .adv_type = ADV_TYPE_IND,
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		//.peer_addr            =
		//.peer_addr_type       =
		.channel_map = ADV_CHNL_ALL, .adv_filter_policy =
				ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, };


/**
 * @brief       A callback event when a gap request (esp_gap_ble_cb_event_t) is received.
 * @param  		event: event type esp_gap_ble_cb_event_t.
 *         		param: gap callback parameter esp_ble_gap_cb_param_t.
 * @return      none
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
		esp_ble_gap_cb_param_t *param)
{
	printf("GAP EVENT = %d\n", event);
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
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
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
static void gatts_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT)
	{
		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(
		TEST_DEVICE_NAME);
		if (set_dev_name_ret)
		{
			ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x",
					set_dev_name_ret);
		}

		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret)
		{
			ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
		}

		esp_ble_gap_start_advertising(&adv_params);
	}

	if (event == ESP_GATTS_DISCONNECT_EVT)
	{
		esp_ble_gap_start_advertising(&adv_params);
	}

}

/**
 * @brief       Main function that releases existing ble memory and initializes & enable ble controller.
 * 				It also initialize and enable bluedroid register callback for GATT and GAP.
 * 				Registers a GATT profile (We will study about this in further code).
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

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
	;
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__,
				esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__,
				esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_init();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__,
				esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__,
				esp_err_to_name(ret));
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

	ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	return;
}
