#ifndef PUSH_BUTTON_H
#define PUSH_BUTTON_H

#include <esp_err.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PB_TAG        "PB"
#define PB_1_GPIO_PIN (7)
#define PB_2_GPIO_PIN (4)
#define PB_3_GPIO_PIN (5)
#define PB_4_GPIO_PIN (6) 

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t pb_init(void); 

#endif /* PUSH_BUTTON_H */
