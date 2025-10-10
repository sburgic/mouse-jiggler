#ifndef LED_H
#define LED_H

#include <esp_err.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define LED_TAG "LED"

#define LED_GREEN_GPIO_PIN GPIO_NUM_20
#define LED_RED_GPIO_PIN   GPIO_NUM_21

#define LED_COLOR_GREEN (0)
#define LED_COLOR_RED   (1)

#define LED_FLAG_MODE        (0xFC)
#define LED_FLAG_PAIR_STATUS (0xF3)
#define LED_FLAG_ERROR       (0xCF)
#define LED_FLAG_BAT_FULL    (0x3F)

#define LED_MODE_AP          (0x01)
#define LED_MODE_CLIENT      (0x02)
#define LED_PAIRED_OK        (0x08)
#define LED_PAIRED_NOT_OK    (0x04)
#define LED_ERROR_ON         (0x10)
#define LED_ERROR_OFF        (0x00)
#define LED_BAT_ON           (0x80)
#define LED_BAT_OFF          (0x00)

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t led_init(void);
void led_raw_cmd_disable(void);
void led_raw_set(uint8_t color, uint8_t state);
void led_set(uint8_t flag, uint8_t state);

#endif /* LED_H */
