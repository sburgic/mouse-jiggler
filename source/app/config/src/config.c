#include "config.h"

#include "error.h"

#include <esp_log.h>
#include <esp_partition.h>

/*******************************************************************************
 * Data
 ******************************************************************************/

static const esp_partition_t* cfg_part_ptr;
static config_t               cfg_data;

/*******************************************************************************
 * API functions
 ******************************************************************************/

esp_err_t config_init(void)
{
    esp_err_t ret = ESP_FAIL;

    /* Find the partition map in the partition table. */
    cfg_part_ptr = esp_partition_find_first( CONFIG_PARTITION_TYPE
                                           , CONFIG_PARTITION_SUBTYPE
                                           , CONFIG_PARTITION_NAME
                                           );

    if (NULL != cfg_part_ptr)
    {
        ret = esp_partition_read_raw( cfg_part_ptr
                                    , 0
                                    , &cfg_data
                                    , sizeof(config_t)
                                    );
    }

    if (ESP_OK == ret)
    {
        ESP_LOGI(CONFIG_TAG, "Initialization done.");
    }
    else
    {
        ESP_LOGE(CONFIG_TAG, "Failed to read data from partition.");
        error_critical_handler(CONFIG_TAG);
    }

    return ret;
}

esp_err_t config_data_store(void)
{
    esp_err_t ret;

    ret = esp_partition_erase_range( cfg_part_ptr
                                   , CONFIG_PARTITION_OFFSET
                                   , cfg_part_ptr->erase_size
                                   );

    if (ESP_OK == ret)
    {
        ret = esp_partition_write( cfg_part_ptr
                                 , CONFIG_PARTITION_OFFSET
                                 , &cfg_data
                                 , sizeof(config_t)
                                 );
    }

    return ret;
}

esp_err_t config_user_data_clear(void)
{
    uint16_t i;

    /* Set AP mode flag. */
    cfg_data.ap_mode = 1;

    /* Clear idle time. */
    cfg_data.idle_time_s = 0;

    /* Clear password 1. */
    for (i = 0; i < CONFIG_PASSWORD_MAX_SIZE; i++)
    {
        cfg_data.password1[i] = 0;
    }

    /* Clear password 2. */
    for (i = 0; i < CONFIG_PASSWORD_MAX_SIZE; i++)
    {
        cfg_data.password2[i] = 0;
    }

    /* Clear password 3. */
    for (i = 0; i < CONFIG_PASSWORD_MAX_SIZE; i++)
    {
        cfg_data.password3[i] = 0;
    }

    /* Clear password 4. */
    for (i = 0; i < CONFIG_PASSWORD_MAX_SIZE; i++)
    {
        cfg_data.password4[i] = 0;
    }

    return config_data_store();
}

config_t* config_hdl_get(void)
{
    return &cfg_data;
}
