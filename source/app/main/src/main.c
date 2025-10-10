/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "adc.h"
#include "bluetooth.h"
#include "config.h"
#include "error.h"
#include "led.h"
#include "power-latch.h"
#include "push-button.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define MAIN_TAG "MAIN"

/*******************************************************************************
 * API functions
 ******************************************************************************/

void app_main(void)
{
    esp_err_t ret;
    uint16_t  level = 1;

    /* Enable the power regulator. */
    ret = power_latch_init();

    if (ESP_OK == ret)
    {
        /* Initialize LED handler. */
        ret = led_init();
    }

    if (ESP_OK == ret)
    {
        do
        {
            /* Wait until the PB is released. */
            level = gpio_get_level(PB_1_GPIO_PIN);
            vTaskDelay(pdMS_TO_TICKS(100));
        } while (0 != level);
    }

    if (ESP_OK == ret)
    {
        ret = nvs_flash_init();

        if ((ESP_ERR_NVS_NO_FREE_PAGES == ret)
         || (ESP_ERR_NVS_NEW_VERSION_FOUND == ret))
        {
            ret = nvs_flash_erase();

            if (ESP_OK == ret)
            {
                ret = nvs_flash_init();
            }
        }
    }

    if (ESP_OK == ret)
    {
        /* Initialize configuration handler. */
        ret = config_init();
    }

    if (ESP_OK == ret)
    {
        /* Initialize ADC handler. */
        ret = adc_init();
    }

    if (ESP_OK == ret)
    {
        /* Initialize bluetooth handler. */
        ret = bt_init();
    }

    if (ESP_OK == ret)
    {
        ret = pb_init();
    }

    if (ESP_OK == ret)
    {
        ESP_LOGI(MAIN_TAG, "Initialization successful.");
    }
}
