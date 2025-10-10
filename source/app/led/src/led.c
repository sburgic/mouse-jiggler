#include "led.h"

#include "error.h"

#include <driver/gpio.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*******************************************************************************
 * Definitions and types
 ******************************************************************************/

#define LED_TASK_PRIORITY         (2)
#define LED_TASK_STACK_SIZE       (512)
#define LED_TASK_DELAY_ACTIVE_MS  (50)
#define LED_TASK_DELAY_PASSIVE_MS (500)
#define LED_TASK_DELAY_SLEEP_MS   (5000)
#define LED_TASK_SLICES           (8)

/*******************************************************************************
 * Data
 ******************************************************************************/

static TaskHandle_t led_task_hdl     = NULL;
static uint8_t      led_task_cnt     = 0;
static uint8_t      led_flag_bitmask = 0;
static uint8_t      led_task_running = 0;

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

static void led_set_gpio(uint8_t bitmask)
{
    (void) gpio_set_level(LED_RED_GPIO_PIN, bitmask & 0x01);
    (void) gpio_set_level(LED_GREEN_GPIO_PIN, bitmask & 0x02);
};

static void led_task(void* param)
{
    TickType_t last_wake_time;

    (void) param;

    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        if (0 != led_task_running)
        {
            if (LED_TASK_SLICES == led_task_cnt)
            {
                led_task_cnt = 0;
                vTaskDelayUntil( &last_wake_time
                               , pdMS_TO_TICKS(LED_TASK_DELAY_SLEEP_MS)
                               );
            }
            else
            {
                if (0 == (led_task_cnt % 2))
                {
                    led_set_gpio(led_flag_bitmask >> led_task_cnt);
                    vTaskDelayUntil( &last_wake_time
                                   , pdMS_TO_TICKS(LED_TASK_DELAY_ACTIVE_MS)
                                   );
                }
                else
                {
                    vTaskDelayUntil( &last_wake_time
                                   , pdMS_TO_TICKS(LED_TASK_DELAY_PASSIVE_MS)
                                   );
                }

                led_task_cnt++;
            }

            led_set_gpio(0);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/*******************************************************************************
 * API functions
 ******************************************************************************/

esp_err_t led_init(void)
{
    esp_err_t     ret;
    gpio_config_t cfg = {0};

    cfg.intr_type    = GPIO_INTR_DISABLE;            /* Disable interrupt. */
    cfg.mode         = GPIO_MODE_OUTPUT;             /* Set as output. */
    cfg.pin_bit_mask = ((1ULL << LED_GREEN_GPIO_PIN)
                     | (1ULL << LED_RED_GPIO_PIN));  /* Set pin bitmask. */
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;        /* Disable pull-down. */
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;          /* Disable pull-up. */

    ret = gpio_config(&cfg);

    if (ESP_OK == ret)
    {
        xTaskCreate( led_task
                   , "LED_TASK"
                   , LED_TASK_STACK_SIZE
                   , NULL
                   , LED_TASK_PRIORITY
                   , &led_task_hdl
                   );

        if (NULL == led_task_hdl)
        {
            error_critical_handler(LED_TAG);
        }
        else
        {
            ESP_LOGI(LED_TAG, "Initialization done.");
        }
    }

    return ret;
}

void led_raw_cmd_disable(void)
{
    led_task_running = 1;
}

void led_raw_set(uint8_t color, uint8_t state)
{
    if (0 == led_task_running)
    {
        if (LED_COLOR_GREEN == color)
        {
            (void) gpio_set_level(LED_GREEN_GPIO_PIN, state);
        }
        else if (LED_COLOR_RED == color)
        {
            (void) gpio_set_level(LED_RED_GPIO_PIN, state);
        }
        else
        {
            /* Nothing to do here. */
        }
    }
}

void led_set(uint8_t flag, uint8_t state)
{
    led_flag_bitmask &= flag;
    led_flag_bitmask |= state;
}
