#include "push-button.h"

#include "error.h"
#include "led.h"
#include "power-latch.h"

#include <driver/gpio.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PB_TASK_PRIORITY       (2)
#define PB_TASK_STACK_SIZE     (1024)
#define PB_TASK_DELAY_SLEEP_MS (100)

/*******************************************************************************
 * Data
 ******************************************************************************/

static TaskHandle_t pb_task_hdl = NULL;

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

static void pb_task(void* param)
{
    uint8_t pb_state;
    uint8_t pressed_cycles = 0;

    (void) param;

    for (;;)
    {
        pb_state = gpio_get_level(PB_1_GPIO_PIN);

        if (0 != pb_state)
        {
            (void) led_raw_set(LED_COLOR_RED, 1);
            pressed_cycles++;

            /* Valid press/release detected. */
            if (pressed_cycles >= 10)
            {
                /* Turn off the power supply. */
                power_latch_deinit();

                /* Not needed here, but might be required in the future
                 * if the switch has different modes.
                 */
                pressed_cycles = 0;
                (void) led_raw_set(LED_COLOR_RED, 0);

                for (;;)
                {
                    vTaskDelay(pdMS_TO_TICKS(PB_TASK_DELAY_SLEEP_MS));
                }
            }
        }
        else
        {
            pressed_cycles = 0;
            (void) led_raw_set(LED_COLOR_RED, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(PB_TASK_DELAY_SLEEP_MS));
    }
}

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t pb_init(void)
{
    esp_err_t     ret;
    gpio_config_t cfg = {0};

    cfg.intr_type    = GPIO_INTR_DISABLE;       /* Disable interrupt. */
    cfg.mode         = GPIO_MODE_INPUT;         /* Set as input. */
    cfg.pin_bit_mask = (1ULL << PB_1_GPIO_PIN)
                     | (1ULL << PB_2_GPIO_PIN)
                     | (1ULL << PB_3_GPIO_PIN)
                     | (1ULL << PB_4_GPIO_PIN); /* Set pin bitmask. */
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;   /* Disable pull-down. */
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;     /* Disable pull-up. */

    ret = gpio_config(&cfg);

    if (ESP_OK == ret)
    {
        xTaskCreate( pb_task
                   , "PB_TASK"
                   , PB_TASK_STACK_SIZE
                   , NULL
                   , PB_TASK_PRIORITY
                   , &pb_task_hdl
                   );

        if (NULL == pb_task_hdl)
        {
            ret = ESP_FAIL;
        }
    }

    return ret;
}
