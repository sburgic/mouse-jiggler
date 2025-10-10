#include "bluetooth.h"

#include "error.h"
#include "led.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_gatt_defs.h>
#include <esp_hidd.h>
#include <esp_hid_gap.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BT_TASK_PRIORITY   (3)
#define BT_TASK_STACK_SIZE (8192)
#define BT_TASK_DELAY_MS   (50)

#define BT_MODE_BLE (0x01)

typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

/*******************************************************************************
 * Data
 ******************************************************************************/

const unsigned char hidapiReportMap[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,        // USAGE (Mouse)
    0xA1, 0x01,        // COLLECTION (Application)
    0x85, 0x01,        //   REPORT_ID (1)
    0x09, 0x01,        //   USAGE (Pointer)
    0xA1, 0x00,        //   COLLECTION (Physical)
    // ------------------------------------------------- Buttons (Left, Right, Middle, Back, Forward)
    0x05, 0x09,        //     USAGE_PAGE (Button)
    0x19, 0x01,        //     USAGE_MINIMUM (Button 1)
    0x29, 0x05,        //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x05,        //     REPORT_COUNT (5)
    0x81, 0x02,        //     INPUT (Data, Variable, Absolute) ;5 button bits
    // ------------------------------------------------- Padding
    0x75, 0x03,        //     REPORT_SIZE (3)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x81, 0x03,        //     INPUT (Constant, Variable, Absolute) ;3 bit padding
    // ------------------------------------------------- X/Y position, Wheel
    0x05, 0x01,        //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //     USAGE (X)
    0x09, 0x31,        //     USAGE (Y)
    0x09, 0x38,        //     USAGE (Wheel)
    0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,        //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x03,        //     REPORT_COUNT (3)
    0x81, 0x06,        //     INPUT (Data, Variable, Relative) ;3 bytes (X,Y,Wheel)
    // ------------------------------------------------- Horizontal wheel
    0x05, 0x0C,        //     USAGE_PAGE (Consumer Devices)
    0x0A, 0x38, 0x02,  //     USAGE (AC Pan)
    0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,        //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x81, 0x06,        //     INPUT (Data, Var, Rel)
    0xC0,              //   END_COLLECTION
    0xC0               // END_COLLECTION
};

static esp_hid_raw_report_map_t bt_ble_report_maps[] = {
    {
        .data = hidapiReportMap,
        .len = sizeof(hidapiReportMap)
    }
};

static esp_hid_device_config_t bt_ble_hid_config =
{
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "burgiclab HID",
    .manufacturer_name  = "burgiclab",
    .serial_number      = "0000000001",
    .report_maps        = bt_ble_report_maps,
    .report_maps_len    = 1
};

static local_param_t bt_ble_hid_param = {0};

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

static void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    static uint8_t buffer[5] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    buffer[4] = 0;
    esp_hidd_dev_input_set(bt_ble_hid_param.hid_dev, 0, 1, buffer, 5);
}

static void ble_hid_demo_task(void *pvParameters)
{
    uint16_t direction = 0;

    while (1)
    {
        switch (direction)
        {
        case 0: /* Up. */
            send_mouse(0, 0, 0xF6, 0);
            break;
        case 1: /* Right. */
            send_mouse(0, 0x0A, 0, 0);
            break;
        case 2: /* Down. */
            send_mouse(0, 0, 0x0A, 0);
            break;
        case 3: /* Left. */
            send_mouse(0, 0xF6, 0, 0);
            break;
        }

        /* Wait 10 second. */
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        /* Move to the next direction. */
        direction = (direction + 1) % 4;
    }
}

static void ble_hid_task_start_up(void)
{
    xTaskCreate(ble_hid_demo_task, "ble_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3,
                &bt_ble_hid_param.task_hdl);
}

static void ble_hid_task_shut_down(void)
{
    if (bt_ble_hid_param.task_hdl) {
        vTaskDelete(bt_ble_hid_param.task_hdl);
        bt_ble_hid_param.task_hdl = NULL;
    }
}

static void bt_ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event) {
    case ESP_HIDD_START_EVENT: {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        ESP_LOGI(TAG, "CONNECT");
        ble_hid_task_start_up();//todo: this should be on auth_complete (in GAP)
        led_raw_set(LED_COLOR_GREEN, 1);
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT: {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        led_raw_set(LED_COLOR_GREEN, 0);
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG, "STOP");
        led_raw_set(LED_COLOR_GREEN, 0);
        break;
    }
    default:
        break;
    }
    return;
}

/*******************************************************************************
 * API functions
 ******************************************************************************/

esp_err_t bt_init(void)
{
    esp_err_t ret;

    ret = esp_hid_gap_init(BT_MODE_BLE);

    if (ESP_OK == ret)
    {
        ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, bt_ble_hid_config.device_name);
    }

    if (ESP_OK == ret)
    {
        ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler);
    }

    if (ESP_OK == ret)
    {
        ret = esp_hidd_dev_init(&bt_ble_hid_config, ESP_HID_TRANSPORT_BLE, bt_ble_hidd_event_callback, &bt_ble_hid_param.hid_dev);
    }

    return ret;
}
