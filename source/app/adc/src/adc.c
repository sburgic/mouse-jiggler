#include "adc.h"

#include "error.h"

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ADC_TASK_PRIORITY          (1)
#define ADC_TASK_STACK_SIZE        (4096)
#define ADC_TASK_DELAY_DEFAULT_MS  (100)
#define ADC_EVENT_COLLECT_DONE_BIT (1 << 0)
#define ADC_TASK_CONV_FRAME_SIZE   (256 * SOC_ADC_DIGI_RESULT_BYTES)
#define ADC_TASK_NO_DATA_CNT_MAX   (10)

/*******************************************************************************
 * Data
 ******************************************************************************/

static TaskHandle_t            adc_task_hdl;
static EventGroupHandle_t      adc_event_group     = NULL;
static adc_continuous_handle_t adc_cont_hdl        = NULL;
static adc_cali_handle_t       adc_cal_hdl         = NULL;
static uint16_t                adc_last_raw        = ADC_DEFAULT_VALUE;
static uint16_t                adc_last_calculated = 0;
static uint8_t                 adc_calibrated      = 0;

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

static bool IRAM_ATTR adc_conv_done_cb
                            ( adc_continuous_handle_t          adc_cont_hdl
                            , const adc_continuous_evt_data_t* event_data
                            , void*                            user_data
                            )
{
    BaseType_t yield = pdFALSE;

    (void) adc_cont_hdl;
    (void) event_data;
    (void) user_data;

    /* Notify that ADC continuous driver has done
     * enough number of conversions.
     */
    vTaskNotifyGiveFromISR(adc_task_hdl, &yield);

    return (pdTRUE == yield);
}

static esp_err_t adc_cont_init(void)
{
    esp_err_t                 ret;
    adc_digi_pattern_config_t adc_pattern  =
    {
        .atten     = ADC_ATTEN_DB_11,
        .channel   = ADC_CHANNEL_0 & 0x7,
        .unit      = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_handle_cfg_t adc_config =
    {
        .max_store_buf_size = 2 * ADC_TASK_CONV_FRAME_SIZE,
        /* Must be divisible by SOC_ADC_DIGI_DATA_BYTES_PER_CONV. */
        .conv_frame_size = ADC_TASK_CONV_FRAME_SIZE
    };
    adc_continuous_config_t dig_cfg =
    {
        .sample_freq_hz = 1024, /* Must be higher than
                                 * SOC_ADC_SAMPLE_FREQ_THRES_LOW.
                                 */
        .conv_mode   = ADC_CONV_SINGLE_UNIT_1,
        .format      = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .adc_pattern = &adc_pattern,
        .pattern_num = 1 /* Total number of ADC channels used. */
    };

    ret = adc_continuous_new_handle(&adc_config, &adc_cont_hdl);

    if (ESP_OK == ret)
    {
        ret = adc_continuous_config(adc_cont_hdl, &dig_cfg);
    }

    if (NULL == adc_cont_hdl)
    {
        ret = ESP_FAIL;
    }

    return ret;
}

static esp_err_t adc_cal_init(void)
{
    esp_err_t                       ret;
    adc_cali_curve_fitting_config_t cal_cfg =
    {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ret = adc_cali_create_scheme_curve_fitting(&cal_cfg, &adc_cal_hdl);

    if (ESP_OK == ret)
    {
        ESP_LOGI(ADC_TAG, "Calibration done.");
        adc_calibrated = 1;
    }
    else if (ESP_ERR_NOT_SUPPORTED == ret)
    {
        ESP_LOGW(ADC_TAG, "eFuse not burnt, skipping software calibration.");
        ret = ESP_OK;
    }
    else
    {
        ESP_LOGE(ADC_TAG, "No memory to perform calibration.");
    }

    return ret;
}

static void adc_task(void* param)
{
    esp_err_t                ret;
    uint32_t                 read = 0;
    uint16_t                 i;
    uint32_t                 raw;
    uint8_t                  result[ADC_TASK_CONV_FRAME_SIZE] = {0};
    uint8_t                  no_data_cnt = 0;
    adc_digi_output_data_t*  data_ptr;
    adc_continuous_evt_cbs_t adc_cbs = {.on_conv_done = adc_conv_done_cb};

    ret  = adc_cont_init();
    ret |= adc_cal_init();
    ret |= adc_continuous_register_event_callbacks( adc_cont_hdl
                                                  , &adc_cbs
                                                  , NULL
                                                  );
    ret |= adc_continuous_start(adc_cont_hdl);

    if (ESP_OK == ret)
    {
        for (;;)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            ret = adc_continuous_read( adc_cont_hdl
                                     , result
                                     , ADC_TASK_CONV_FRAME_SIZE
                                     , &read
                                     , 0
                                     );

            if (ret == ESP_OK)
            {
                no_data_cnt = 0;
                raw         = 0;

                for (i = 0; i < read; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    data_ptr = (adc_digi_output_data_t*) &result[i];
                    raw += data_ptr->type2.data;
                }

                adc_last_raw = (uint16_t) \
                               (raw / (i / SOC_ADC_DIGI_RESULT_BYTES));

                if (0 != adc_calibrated)
                {
                    adc_cali_raw_to_voltage( adc_cal_hdl
                                           , adc_last_raw
                                           , (int*) &adc_last_calculated);
                }
                else
                {
                    adc_last_calculated = \
                        (uint16_t)((uint32_t)(adc_last_raw * 2500) / 4095);
                }

                /* Notify any waiting task that a full cycle is completed. */
                xEventGroupSetBits( adc_event_group
                                  , ADC_EVENT_COLLECT_DONE_BIT
                                  );
            }
            else
            {
                no_data_cnt++;

                if (no_data_cnt >= ADC_TASK_NO_DATA_CNT_MAX)
                {
                    adc_last_raw = ADC_DEFAULT_VALUE;
                }
            }
        }
    }
}

static esp_err_t adc_ctl_pin_init(void)
{
    esp_err_t     ret;
    gpio_config_t cfg = {0};

    cfg.intr_type    = GPIO_INTR_DISABLE;     /* Disable interrupt. */
    cfg.mode         = GPIO_MODE_OUTPUT;      /* Set as output. */
    cfg.pin_bit_mask = (1ULL << ADC_CTL_GPIO_PIN);
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE; /* Disable pull-down. */
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;   /* Disable pull-up. */

    ret = gpio_config(&cfg);

    if (ESP_OK == ret)
    {
        ret = gpio_set_level(ADC_CTL_GPIO_PIN, 1);
    }

    return ret;
}

/*******************************************************************************
 * API functions
 ******************************************************************************/

esp_err_t adc_init(void)
{
    esp_err_t   ret;
    EventBits_t event_bits;

    ret = adc_ctl_pin_init();

    if (ESP_OK == ret)
    {
        xTaskCreate( &adc_task
                   , "ADC_TASK"
                   , ADC_TASK_STACK_SIZE
                   , NULL
                   , ADC_TASK_PRIORITY
                   , &adc_task_hdl
                   );

        if (NULL == adc_task_hdl)
        {
            error_critical_handler(ADC_TAG);
        }
        else
        {
            adc_event_group = xEventGroupCreate();

            if (NULL == adc_event_group)
            {
                error_critical_handler(ADC_TAG);
            }
            else
            {
                /* Ensure that first cycle of full measurement has passed. */
                event_bits = xEventGroupWaitBits( adc_event_group
                                                , ADC_EVENT_COLLECT_DONE_BIT
                                                , pdTRUE
                                                , pdTRUE
                                                , pdMS_TO_TICKS(5000)
                                                );

                if (0 != (event_bits & ADC_EVENT_COLLECT_DONE_BIT))
                {
                    ESP_LOGI(ADC_TAG, "Initialization done.");
                }
                else
                {
                    error_critical_handler(ADC_TAG);
                }
            }
        }
    }

    return ret;
}

esp_err_t adc_deinit(void)
{
    esp_err_t ret;

    ret  = gpio_set_level(ADC_CTL_GPIO_PIN, 0);
    ret |= gpio_reset_pin(ADC_CTL_GPIO_PIN);

    return ret;
}

uint16_t adc_raw_get(void)
{
    return adc_last_raw;
}

uint16_t adc_mv_get(void)
{
    return (adc_last_calculated << 1);
}
