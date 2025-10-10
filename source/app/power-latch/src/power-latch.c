#include "power-latch.h"

#include "error.h"

#include <driver/gpio.h>
#include <esp_log.h>

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t power_latch_init(void)
{
    esp_err_t     ret;
    gpio_config_t cfg = {0};

    cfg.intr_type    = GPIO_INTR_DISABLE;              /* Disable interrupt. */
    cfg.mode         = GPIO_MODE_OUTPUT;               /* Set as output. */
    cfg.pin_bit_mask = (1ULL << POWER_LATCH_GPIO_PIN); /* Set pin bitmask. */
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;          /* Disable pull-down. */
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;            /* Disable pull-up. */

    ret = gpio_config(&cfg);

    if (ESP_OK == ret)
    {
        ret = gpio_set_level(POWER_LATCH_GPIO_PIN, 1);
    }

    if (ESP_OK == ret)
    {
        ESP_LOGI(POWER_LATCH_TAG, "Power latch initialized.");
    }
    else
    {
        error_critical_handler(POWER_LATCH_TAG);
    }

    return ret;
}

esp_err_t power_latch_deinit(void)
{
    return gpio_set_level(POWER_LATCH_GPIO_PIN, 0);
}
