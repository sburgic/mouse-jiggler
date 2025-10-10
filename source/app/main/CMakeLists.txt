idf_component_register(SRCS "src/main.c"
                       # ESP-IDF components.
                       PRIV_REQUIRES driver
                       PRIV_REQUIRES esp_timer
                       PRIV_REQUIRES nvs_flash
                       # Project components.
                       PRIV_REQUIRES adc
                       PRIV_REQUIRES bluetooth
                       PRIV_REQUIRES config
                       PRIV_REQUIRES error
                       PRIV_REQUIRES led
                       PRIV_REQUIRES power-latch
                       PRIV_REQUIRES push-button)
