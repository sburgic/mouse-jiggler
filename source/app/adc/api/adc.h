#ifndef ADC_H
#define ADC_H

#include <esp_err.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ADC_TAG           "ADC"
#define ADC_GPIO_PIN      GPIO_NUM_0
#define ADC_CTL_GPIO_PIN  GPIO_NUM_3
#define ADC_DEFAULT_VALUE (0)

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t adc_init(void);
esp_err_t adc_deinit(void);
uint16_t adc_raw_get(void);
uint16_t adc_mv_get(void);

#endif /* ADC_H */
