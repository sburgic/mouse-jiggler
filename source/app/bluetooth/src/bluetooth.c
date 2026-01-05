/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bluetooth.h"

#include "error.h"
#include "led.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>
#include <esp_hid_common.h>
#include <esp_hidd.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BT_MODE_BLE (0x01)

/* How often to send the "jiggle". */
#define BT_JIGGLE_PERIOD_MS (1000)

/* BLE connection parameters (interval in 1.25ms units, timeout in 10ms units). */
#define BT_CONN_MIN_ITVL (0x0030) /* 60ms. */
#define BT_CONN_MAX_ITVL (0x0030) /* 60ms. */
#define BT_CONN_LATENCY  (4)      /* Skip up to 4 intervals. */
#define BT_CONN_TIMEOUT  (600)    /* 6s. */

/* Retry delay and maximum number of retries for re-applying our conn params. */
#define BT_CONN_RETRY_DELAY_MS (5000)
#define BT_CONN_RETRY_MAX      (5)

/* Uncomment to print all devices that were seen during a scan. */
#define BT_GAP_DBG_PRINTF(...) /* printf(__VA_ARGS__). */

/*******************************************************************************
 * Types
 ******************************************************************************/

typedef struct bt_hid_scan_result_s
{
    struct bt_hid_scan_result_s* next;

    esp_bd_addr_t       bda;
    const char*         name;
    int8_t              rssi;
    esp_hid_usage_t     usage;
    esp_hid_transport_t transport;

    struct
    {
        esp_ble_addr_type_t addr_type;
        uint16_t            appearance;
    } ble;
} bt_hid_scan_result_t;

typedef struct
{
    TaskHandle_t    task_hdl;
    esp_hidd_dev_t* hid_dev;
    uint8_t         protocol_mode;
    uint8_t*        buffer;
} bt_local_param_t;

/*******************************************************************************
 * Private function prototypes
 ******************************************************************************/

static void bt_ble_adv_start(void);
static void bt_ble_cb_send(void);

static void bt_ble_conn_params_request(void);
static void bt_ble_conn_params_retry_timer_cb(TimerHandle_t xTimer);
static void bt_ble_conn_params_retry_arm(uint32_t delay_ms);

static esp_err_t bt_ble_gap_adv_init(uint16_t appearance, const char* device_name);
static esp_err_t bt_ble_gap_init(void);
static void bt_ble_gap_event_handle(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

static void bt_ble_jiggle_timer_cb(TimerHandle_t xTimer);
static void bt_ble_scan_result_add(esp_bd_addr_t bda,
                                  esp_ble_addr_type_t addr_type,
                                  uint16_t appearance,
                                  uint8_t* name,
                                  uint8_t name_len,
                                  int rssi);
static void bt_ble_task_demo(void* pv_parameters);
static void bt_ble_task_start(void);
static void bt_ble_task_stop(void);

static void bt_device_result_handle(struct ble_scan_result_evt_param* scan_rst);
static void bt_hidd_event_callback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);
static esp_err_t bt_low_level_init(uint8_t mode);

static void bt_mouse_send(uint8_t buttons, char dx, char dy, char wheel);
static bt_hid_scan_result_t* bt_scan_result_find(esp_bd_addr_t bda, bt_hid_scan_result_t* results);
static const char* bt_str_ble_key_type(esp_ble_key_type_t key_type);

/* Our GATTS wrapper that calls both: ESP HID handler + our hook. */
static void bt_gatts_event_handler(esp_gatts_cb_event_t event,
                                  esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t* param);

/*******************************************************************************
 * Data
 ******************************************************************************/

static const char* bt_tag = "BT";

static SemaphoreHandle_t bt_ble_hidh_cb_semaphore = NULL;

static bt_hid_scan_result_t* bt_ble_scan_results = NULL;
static size_t                bt_num_ble_scan_results = 0;

static bt_local_param_t bt_ble_hid_param = {0};

/* Peer address for connection parameter updates. */
static esp_bd_addr_t bt_peer_bda = {0};
static bool          bt_peer_valid = false;

/* Timer that wakes the task for the "jiggle". */
static TimerHandle_t bt_jiggle_timer = NULL;

/* Connection parameters retry logic. */
static TimerHandle_t bt_conn_retry_timer = NULL;
static bool          bt_conn_update_pending = false;
static uint8_t       bt_conn_retry_left = 0;

/* Protect against multiple CONNECT notifications. */
static bool bt_gatts_connected = false;

static const unsigned char bt_hidapi_report_map[] =
{
    0x05, 0x01,        /* USAGE_PAGE (Generic Desktop). */
    0x09, 0x02,        /* USAGE (Mouse). */
    0xA1, 0x01,        /* COLLECTION (Application). */
    0x85, 0x01,        /* REPORT_ID (1). */
    0x09, 0x01,        /* USAGE (Pointer). */
    0xA1, 0x00,        /* COLLECTION (Physical). */

    0x05, 0x09,        /* USAGE_PAGE (Button). */
    0x19, 0x01,        /* USAGE_MINIMUM (Button 1). */
    0x29, 0x05,        /* USAGE_MAXIMUM (Button 5). */
    0x15, 0x00,        /* LOGICAL_MINIMUM (0). */
    0x25, 0x01,        /* LOGICAL_MAXIMUM (1). */
    0x75, 0x01,        /* REPORT_SIZE (1). */
    0x95, 0x05,        /* REPORT_COUNT (5). */
    0x81, 0x02,        /* INPUT (Data, Variable, Absolute). */

    0x75, 0x03,        /* REPORT_SIZE (3). */
    0x95, 0x01,        /* REPORT_COUNT (1). */
    0x81, 0x03,        /* INPUT (Constant, Variable, Absolute). */

    0x05, 0x01,        /* USAGE_PAGE (Generic Desktop). */
    0x09, 0x30,        /* USAGE (X). */
    0x09, 0x31,        /* USAGE (Y). */
    0x09, 0x38,        /* USAGE (Wheel). */
    0x15, 0x81,        /* LOGICAL_MINIMUM (-127). */
    0x25, 0x7F,        /* LOGICAL_MAXIMUM (127). */
    0x75, 0x08,        /* REPORT_SIZE (8). */
    0x95, 0x03,        /* REPORT_COUNT (3). */
    0x81, 0x06,        /* INPUT (Data, Variable, Relative). */

    0x05, 0x0C,        /* USAGE_PAGE (Consumer Devices). */
    0x0A, 0x38, 0x02,  /* USAGE (AC Pan). */
    0x15, 0x81,        /* LOGICAL_MINIMUM (-127). */
    0x25, 0x7F,        /* LOGICAL_MAXIMUM (127). */
    0x75, 0x08,        /* REPORT_SIZE (8). */
    0x95, 0x01,        /* REPORT_COUNT (1). */
    0x81, 0x06,        /* INPUT (Data, Var, Rel). */

    0xC0,              /* END_COLLECTION. */
    0xC0               /* END_COLLECTION. */
};

static esp_hid_raw_report_map_t bt_ble_report_maps[] =
{
    {
        .data = bt_hidapi_report_map,
        .len  = sizeof(bt_hidapi_report_map),
    }
};

static esp_hid_device_config_t bt_ble_hid_config =
{
    .vendor_id         = 0x16C0,
    .product_id        = 0x05DF,
    .version           = 0x0100,
    .device_name       = "burgiclab HID",
    .manufacturer_name = "burgiclab",
    .serial_number     = "0000000001",
    .report_maps       = bt_ble_report_maps,
    .report_maps_len   = 1,
};

/*******************************************************************************
 * Private functions
 ******************************************************************************/

static void bt_ble_adv_start(void)
{
    static esp_ble_adv_params_t bt_hidd_adv_params =
    {
        .adv_int_min       = 0x20,
        .adv_int_max       = 0x30,
        .adv_type          = ADV_TYPE_IND,
        .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    (void)esp_ble_gap_start_advertising(&bt_hidd_adv_params);
}

static void bt_ble_cb_send(void)
{
    (void)xSemaphoreGive(bt_ble_hidh_cb_semaphore);
}

static void bt_ble_conn_params_retry_arm(uint32_t delay_ms)
{
    if (bt_conn_retry_timer == NULL)
    {
        bt_conn_retry_timer = xTimerCreate(
            "bt_conn_retry",
            pdMS_TO_TICKS(delay_ms),
            pdFALSE,
            NULL,
            bt_ble_conn_params_retry_timer_cb
        );

        if (bt_conn_retry_timer == NULL)
        {
            ESP_LOGE(bt_tag, "xTimerCreate failed for conn retry timer.");
            return;
        }
    }

    (void)xTimerStop(bt_conn_retry_timer, 0);
    (void)xTimerChangePeriod(bt_conn_retry_timer, pdMS_TO_TICKS(delay_ms), 0);
    (void)xTimerStart(bt_conn_retry_timer, 0);
}

static void bt_ble_conn_params_retry_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (!bt_peer_valid)
    {
        return;
    }

    if (bt_conn_retry_left == 0)
    {
        ESP_LOGW(bt_tag, "Conn params retry budget exhausted.");
        return;
    }

    ESP_LOGI(bt_tag, "Retrying conn params update (%u left).", (unsigned)bt_conn_retry_left);
    bt_ble_conn_params_request();
}

static void bt_ble_conn_params_request(void)
{
    if (!bt_peer_valid)
    {
        ESP_LOGW(bt_tag, "No peer address yet; cannot update conn params.");
        return;
    }

    if (bt_conn_update_pending)
    {
        ESP_LOGW(bt_tag, "Conn params update is pending; skipping new request.");
        return;
    }

    esp_ble_conn_update_params_t p;
    memset(&p, 0, sizeof(p));

    memcpy(p.bda, bt_peer_bda, sizeof(esp_bd_addr_t));
    p.min_int = BT_CONN_MIN_ITVL;
    p.max_int = BT_CONN_MAX_ITVL;
    p.latency = BT_CONN_LATENCY;
    p.timeout = BT_CONN_TIMEOUT;

    bt_conn_update_pending = true;

    esp_err_t ret = esp_ble_gap_update_conn_params(&p);
    if (ret != ESP_OK)
    {
        bt_conn_update_pending = false;
        ESP_LOGW(bt_tag, "esp_ble_gap_update_conn_params failed: %d.", ret);
        return;
    }

    ESP_LOGI(
        bt_tag,
        "Requested conn params: int=%u..%u (1.25ms), lat=%u, timeout=%u (10ms).",
        (unsigned)p.min_int,
        (unsigned)p.max_int,
        (unsigned)p.latency,
        (unsigned)p.timeout
    );

    if (bt_conn_retry_left > 0)
    {
        bt_conn_retry_left--;
    }
}

static esp_err_t bt_ble_gap_adv_init(uint16_t appearance, const char* device_name)
{
    esp_err_t ret;

    static const uint8_t bt_hidd_service_uuid128[] =
    {
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
    };

    static uint8_t bt_manufacturer_data[] = { 0xE5, 0x02 };

    esp_ble_adv_data_t ble_adv_data =
    {
        .set_scan_rsp        = false,
        .include_name        = false,
        .include_txpower     = true,
        .min_interval        = 0x0006,
        .max_interval        = 0x0010,
        .appearance          = appearance,
        .manufacturer_len    = 0,
        .p_manufacturer_data = NULL,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = sizeof(bt_hidd_service_uuid128),
        .p_service_uuid      = (uint8_t*)bt_hidd_service_uuid128,
        .flag                = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    };

    esp_ble_adv_data_t ble_scan_rsp_data =
    {
        .set_scan_rsp        = true,
        .include_name        = true,
        .include_txpower     = false,
        .manufacturer_len    = sizeof(bt_manufacturer_data),
        .p_manufacturer_data = (uint8_t*)bt_manufacturer_data,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = 0,
        .p_service_uuid      = NULL,
        .flag                = 0,
    };

    esp_ble_auth_req_t auth_req = ESP_IO_CAP_NONE;
    esp_ble_io_cap_t   iocap = ESP_IO_CAP_NONE;
    uint8_t            init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t            rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t            key_size = 16;
    uint32_t           passkey  = 1234;

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param AUTHEN_REQ_MODE failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param IOCAP_MODE failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param SET_INIT_KEY failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param SET_RSP_KEY failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param MAX_KEY_SIZE failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_security_param SET_STATIC_PASSKEY failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_set_device_name(device_name);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP set_device_name failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_config_adv_data(&ble_adv_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP config_adv_data failed: %d.", ret);
        return ret;
    }

    ret = esp_ble_gap_config_adv_data(&ble_scan_rsp_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "GAP config_adv_data failed: %d.", ret);
        return ret;
    }

    return ESP_OK;
}

static esp_err_t bt_ble_gap_init(void)
{
    esp_err_t ret = esp_ble_gap_register_callback(bt_ble_gap_event_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(bt_tag, "esp_ble_gap_register_callback failed: %d.", ret);
        return ret;
    }

    return ESP_OK;
}

/* GAP callback does not provide CONNECT/DISCONNECT in this configuration; use GATTS for that. */
static void bt_ble_gap_event_handle(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        {
            bt_conn_update_pending = false;

            const uint16_t got_int = (uint16_t)param->update_conn_params.conn_int;
            const uint16_t got_lat = (uint16_t)param->update_conn_params.latency;
            const uint16_t got_to  = (uint16_t)param->update_conn_params.timeout;

            ESP_LOGI(
                bt_tag,
                "Conn params updated: status=%d, int=%u, lat=%u, timeout=%u",
                (int)param->update_conn_params.status,
                (unsigned)got_int,
                (unsigned)got_lat,
                (unsigned)got_to
            );

            if (bt_peer_valid &&
                (got_int != BT_CONN_MIN_ITVL ||
                 got_lat != BT_CONN_LATENCY ||
                 got_to  != BT_CONN_TIMEOUT))
            {
                if (bt_conn_retry_left > 0)
                {
                    ESP_LOGW(bt_tag, "Central changed params; scheduling re-apply.");
                    bt_ble_conn_params_retry_arm(BT_CONN_RETRY_DELAY_MS);
                }
                else
                {
                    ESP_LOGW(bt_tag, "Central changed params; no retries left.");
                }
            }

            break;
        }

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        {
            ESP_LOGV(bt_tag, "BLE GAP EVENT SCAN_PARAM_SET_COMPLETE.");
            bt_ble_cb_send();
            break;
        }

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            esp_ble_gap_cb_param_t* scan_result = param;

            switch (scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    bt_device_result_handle(&scan_result->scan_rst);
                    break;

                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    ESP_LOGV(bt_tag, "BLE GAP EVENT SCAN DONE: %d.", scan_result->scan_rst.num_resps);
                    bt_ble_cb_send();
                    break;

                default:
                    break;
            }
            break;
        }

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        {
            ESP_LOGV(bt_tag, "BLE GAP EVENT SCAN CANCELED.");
            break;
        }

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
        {
            if (!param->ble_security.auth_cmpl.success)
            {
                ESP_LOGE(bt_tag, "BLE GAP AUTH ERROR: 0x%x.", param->ble_security.auth_cmpl.fail_reason);
            }
            else
            {
                ESP_LOGI(bt_tag, "BLE GAP AUTH SUCCESS.");
            }
            break;
        }

        case ESP_GAP_BLE_KEY_EVT:
        {
            ESP_LOGI(bt_tag, "BLE GAP KEY type = %s.", bt_str_ble_key_type(param->ble_security.ble_key.key_type));
            break;
        }

        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        {
            ESP_LOGI(bt_tag, "BLE GAP PASSKEY_NOTIF passkey:%" PRIu32 ".", param->ble_security.key_notif.passkey);
            break;
        }

        case ESP_GAP_BLE_NC_REQ_EVT:
        {
            ESP_LOGI(bt_tag, "BLE GAP NC_REQ passkey:%" PRIu32 ".", param->ble_security.key_notif.passkey);
            esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
            break;
        }

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        {
            ESP_LOGI(bt_tag, "BLE GAP PASSKEY_REQ.");
            break;
        }

        case ESP_GAP_BLE_SEC_REQ_EVT:
        {
            ESP_LOGI(bt_tag, "BLE GAP SEC_REQ.");
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        }

        default:
            break;
    }
}

static void bt_ble_jiggle_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (bt_ble_hid_param.task_hdl != NULL)
    {
        (void)xTaskNotifyGive(bt_ble_hid_param.task_hdl);
    }
}

static void bt_ble_scan_result_add(esp_bd_addr_t bda,
                                  esp_ble_addr_type_t addr_type,
                                  uint16_t appearance,
                                  uint8_t* name,
                                  uint8_t name_len,
                                  int rssi)
{
    if (bt_scan_result_find(bda, bt_ble_scan_results))
    {
        ESP_LOGW(bt_tag, "Result already exists.");
        return;
    }

    bt_hid_scan_result_t* r = (bt_hid_scan_result_t*)malloc(sizeof(bt_hid_scan_result_t));
    if (r == NULL)
    {
        ESP_LOGE(bt_tag, "Malloc scan result failed.");
        return;
    }

    r->next      = NULL;
    r->name      = NULL;
    r->rssi      = (int8_t)rssi;
    r->transport = ESP_HID_TRANSPORT_BLE;
    r->usage     = esp_hid_usage_from_appearance(appearance);

    memcpy(r->bda, bda, sizeof(esp_bd_addr_t));
    r->ble.appearance = appearance;
    r->ble.addr_type  = addr_type;

    if (name_len && name)
    {
        char* name_s = (char*)malloc((size_t)name_len + 1U);
        if (name_s == NULL)
        {
            free(r);
            ESP_LOGE(bt_tag, "Malloc result name failed.");
            return;
        }

        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = (const char*)name_s;
    }

    r->next = bt_ble_scan_results;
    bt_ble_scan_results = r;
    bt_num_ble_scan_results++;
}

/* Task that sleeps until notified, then sends a mouse move and sleeps again. */
static void bt_ble_task_demo(void* pv_parameters)
{
    uint16_t direction = 0;

    (void)pv_parameters;

    while (1)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        switch (direction)
        {
            case 0:
                bt_mouse_send(0, 0, (char)0xF6, 0);
                break;
            case 1:
                bt_mouse_send(0, (char)0x0A, 0, 0);
                break;
            case 2:
                bt_mouse_send(0, 0, (char)0x0A, 0);
                break;
            case 3:
                bt_mouse_send(0, (char)0xF6, 0, 0);
                break;
            default:
                break;
        }

        direction = (uint16_t)((direction + 1U) % 4U);
    }
}

static void bt_ble_task_start(void)
{
    if (bt_ble_hid_param.task_hdl == NULL)
    {
        xTaskCreate(
            bt_ble_task_demo,
            "bt_ble_task_demo",
            2 * 1024,
            NULL,
            configMAX_PRIORITIES - 3,
            &bt_ble_hid_param.task_hdl
        );
    }

    if (bt_jiggle_timer == NULL)
    {
        bt_jiggle_timer = xTimerCreate(
            "bt_jiggle",
            pdMS_TO_TICKS(BT_JIGGLE_PERIOD_MS),
            pdTRUE,
            NULL,
            bt_ble_jiggle_timer_cb
        );

        if (bt_jiggle_timer == NULL)
        {
            ESP_LOGE(bt_tag, "xTimerCreate failed.");
            return;
        }
    }

    (void)xTimerStart(bt_jiggle_timer, 0);
}

static void bt_ble_task_stop(void)
{
    if (bt_jiggle_timer != NULL)
    {
        (void)xTimerStop(bt_jiggle_timer, 0);
    }

    if (bt_ble_hid_param.task_hdl)
    {
        vTaskDelete(bt_ble_hid_param.task_hdl);
        bt_ble_hid_param.task_hdl = NULL;
    }
}

static void bt_device_result_handle(struct ble_scan_result_evt_param* scan_rst)
{
    uint16_t uuid = 0;
    uint16_t appearance = 0;
    char     name[64] = {0};

    uint8_t  uuid_len = 0;
    uint8_t* uuid_d = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_16SRV_CMPL, &uuid_len);
    if (uuid_d != NULL && uuid_len)
    {
        uuid = (uint16_t)(uuid_d[0] + (uuid_d[1] << 8));
    }

    uint8_t  appearance_len = 0;
    uint8_t* appearance_d = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_APPEARANCE, &appearance_len);
    if (appearance_d != NULL && appearance_len)
    {
        appearance = (uint16_t)(appearance_d[0] + (appearance_d[1] << 8));
    }

    uint8_t  adv_name_len = 0;
    uint8_t* adv_name = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
    if (adv_name == NULL)
    {
        adv_name = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
    }

    if (adv_name != NULL && adv_name_len)
    {
        memcpy(name, adv_name, adv_name_len);
        name[adv_name_len] = 0;
    }

    BT_GAP_DBG_PRINTF("BLE: " ESP_BD_ADDR_STR ", ", ESP_BD_ADDR_HEX(scan_rst->bda));
    BT_GAP_DBG_PRINTF("RSSI: %d, ", scan_rst->rssi);
    BT_GAP_DBG_PRINTF("UUID: 0x%04x, ", uuid);
    BT_GAP_DBG_PRINTF("APPEARANCE: 0x%04x, ", appearance);
    if (adv_name_len)
    {
        BT_GAP_DBG_PRINTF(", NAME: '%s'", name);
    }
    BT_GAP_DBG_PRINTF("\n");

    if (uuid == ESP_GATT_UUID_HID_SVC)
    {
        bt_ble_scan_result_add(
            scan_rst->bda,
            scan_rst->ble_addr_type,
            appearance,
            adv_name,
            adv_name_len,
            scan_rst->rssi
        );
    }
}

static void bt_hidd_event_callback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    esp_hidd_event_t   event = (esp_hidd_event_t)id;
    static const char* tag   = "HID_DEV_BLE";

    (void)handler_args;
    (void)base;
    (void)event_data;

    switch (event)
    {
        case ESP_HIDD_START_EVENT:
            ESP_LOGI(tag, "START");
            bt_ble_adv_start();
            break;

        case ESP_HIDD_CONNECT_EVENT:
            ESP_LOGI(tag, "CONNECT");
            /* Do NOT call bt_ble_conn_params_request() here:
             * HIDD CONNECT can come before we have peer BDA (from GATTS).
             * We request params in bt_on_gatts_event() once peer is known.
             */
            bt_ble_task_start();
            led_raw_set(LED_COLOR_GREEN, 1);
            break;

        case ESP_HIDD_DISCONNECT_EVENT:
            ESP_LOGI(tag, "DISCONNECT");
            bt_ble_task_stop();
            bt_ble_adv_start();
            led_raw_set(LED_COLOR_GREEN, 0);
            break;

        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(tag, "STOP");
            bt_ble_task_stop();
            led_raw_set(LED_COLOR_GREEN, 0);
            break;

        default:
            break;
    }
}

static esp_err_t bt_low_level_init(uint8_t mode)
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = mode;
#endif

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret)
    {
        ESP_LOGE(bt_tag, "esp_bt_controller_mem_release failed: %d.", ret);
        return ret;
    }

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(bt_tag, "esp_bt_controller_init failed: %d.", ret);
        return ret;
    }

    ret = esp_bt_controller_enable(mode);
    if (ret)
    {
        ESP_LOGE(bt_tag, "esp_bt_controller_enable failed: %d.", ret);
        return ret;
    }

    ret = esp_bt_sleep_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGW(bt_tag, "esp_bt_sleep_enable failed: %d.", ret);
    }
    else
    {
        ESP_LOGI(bt_tag, "BT sleep enabled (controller power save).");
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(bt_tag, "esp_bluedroid_init failed: %d.", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(bt_tag, "esp_bluedroid_enable failed: %d.", ret);
        return ret;
    }

    ret = bt_ble_gap_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

static void bt_mouse_send(uint8_t buttons, char dx, char dy, char wheel)
{
    static uint8_t bt_buffer[5] = {0};

    bt_buffer[0] = buttons;
    bt_buffer[1] = (uint8_t)dx;
    bt_buffer[2] = (uint8_t)dy;
    bt_buffer[3] = (uint8_t)wheel;
    bt_buffer[4] = 0;

    esp_hidd_dev_input_set(bt_ble_hid_param.hid_dev, 0, 1, bt_buffer, 5);
}

static bt_hid_scan_result_t* bt_scan_result_find(esp_bd_addr_t bda, bt_hid_scan_result_t* results)
{
    bt_hid_scan_result_t* r = results;

    while (r)
    {
        if (memcmp(bda, r->bda, sizeof(esp_bd_addr_t)) == 0)
        {
            return r;
        }
        r = r->next;
    }

    return NULL;
}

static const char* bt_str_ble_key_type(esp_ble_key_type_t key_type)
{
    switch (key_type)
    {
        case ESP_LE_KEY_NONE:  return "ESP_LE_KEY_NONE";
        case ESP_LE_KEY_PENC:  return "ESP_LE_KEY_PENC";
        case ESP_LE_KEY_PID:   return "ESP_LE_KEY_PID";
        case ESP_LE_KEY_PCSRK: return "ESP_LE_KEY_PCSRK";
        case ESP_LE_KEY_PLK:   return "ESP_LE_KEY_PLK";
        case ESP_LE_KEY_LLK:   return "ESP_LE_KEY_LLK";
        case ESP_LE_KEY_LENC:  return "ESP_LE_KEY_LENC";
        case ESP_LE_KEY_LID:   return "ESP_LE_KEY_LID";
        case ESP_LE_KEY_LCSRK: return "ESP_LE_KEY_LCSRK";
        default:               return "INVALID BLE KEY TYPE";
    }
}

/*******************************************************************************
 * GATTS callback hook.
 *
 * NOTE: You must call this from your existing GATTS callback on connect
 * and disconnect events.
 ******************************************************************************/

void bt_on_gatts_event(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    (void)gatts_if;

    switch (event)
    {
        case ESP_GATTS_CONNECT_EVT:
        {
            if (bt_gatts_connected)
            {
                memcpy(bt_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                bt_peer_valid = true;
                return;
            }

            bt_gatts_connected = true;

            memcpy(bt_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            bt_peer_valid = true;

            bt_conn_retry_left = BT_CONN_RETRY_MAX;
            bt_conn_update_pending = false;

            ESP_LOGI(
                bt_tag,
                "GATTS CONNECT: " ESP_BD_ADDR_STR,
                ESP_BD_ADDR_HEX(param->connect.remote_bda)
            );

            (void)esp_ble_gap_set_prefer_conn_params(
                bt_peer_bda,
                BT_CONN_MIN_ITVL,
                BT_CONN_MAX_ITVL,
                BT_CONN_LATENCY,
                BT_CONN_TIMEOUT
            );

            bt_ble_conn_params_request();

            break;
        }

        case ESP_GATTS_DISCONNECT_EVT:
        {
            ESP_LOGI(bt_tag, "GATTS DISCONNECT reason=0x%02x.", (unsigned)param->disconnect.reason);

            bt_gatts_connected = false;

            bt_peer_valid = false;
            memset(bt_peer_bda, 0, sizeof(bt_peer_bda));

            bt_conn_update_pending = false;
            bt_conn_retry_left = 0;

            if (bt_conn_retry_timer != NULL)
            {
                (void)xTimerStop(bt_conn_retry_timer, 0);
            }

            break;
        }

        default:
            break;
    }
}

/*******************************************************************************
 * GATTS wrapper: call both the HID stack handler and our bt_on_gatts_event().
 ******************************************************************************/

static void bt_gatts_event_handler(esp_gatts_cb_event_t event,
                                  esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t* param)
{
    /* Let the HID device stack handle GATTS first. */
    esp_hidd_gatts_event_handler(event, gatts_if, param);

    /* Then hook connect/disconnect to capture peer and request conn params. */
    bt_on_gatts_event(event, gatts_if, param);
}

/*******************************************************************************
 * API functions
 ******************************************************************************/

esp_err_t bt_init(void)
{
    esp_err_t ret;

    bt_ble_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (bt_ble_hidh_cb_semaphore == NULL)
    {
        ESP_LOGE(bt_tag, "xSemaphoreCreateBinary failed.");
        return ESP_FAIL;
    }

    ret = bt_low_level_init(BT_MODE_BLE);
    if (ret != ESP_OK)
    {
        vSemaphoreDelete(bt_ble_hidh_cb_semaphore);
        bt_ble_hidh_cb_semaphore = NULL;
        return ret;
    }

    ret = bt_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, bt_ble_hid_config.device_name);
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Register our wrapper instead of the raw HID GATTS handler. */
    ret = esp_ble_gatts_register_callback(bt_gatts_event_handler);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = esp_hidd_dev_init(
        &bt_ble_hid_config,
        ESP_HID_TRANSPORT_BLE,
        bt_hidd_event_callback,
        &bt_ble_hid_param.hid_dev
    );
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}
