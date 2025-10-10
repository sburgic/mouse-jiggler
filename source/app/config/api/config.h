#ifndef CONFIG_H
#define CONFIG_H

#include <esp_err.h>

/*******************************************************************************
 * Definitions and types
 ******************************************************************************/

#define CONFIG_TAG               "CFG"
#define CONFIG_PARTITION_NAME    "configuration"
#define CONFIG_PARTITION_TYPE    (0x40)
#define CONFIG_PARTITION_SUBTYPE (0x00)
#define CONFIG_PARTITION_OFFSET  (0)
#define CONFIG_PASSWORD_MAX_SIZE (32)

typedef struct __attribute__ ((aligned(16)))
{
    uint8_t  ap_mode;
    uint16_t idle_time_s;
    uint8_t  password1[CONFIG_PASSWORD_MAX_SIZE];
    uint8_t  password2[CONFIG_PASSWORD_MAX_SIZE];
    uint8_t  password3[CONFIG_PASSWORD_MAX_SIZE];
    uint8_t  password4[CONFIG_PASSWORD_MAX_SIZE];
} config_t;

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t config_init(void);
esp_err_t config_data_store(void);
esp_err_t config_user_data_clear(void);
config_t* config_hdl_get(void);

#endif /* CONFIG_H */
