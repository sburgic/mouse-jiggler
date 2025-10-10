idf_component_register(SRCS "src/adc.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES driver
                       PRIV_REQUIRES esp_adc
                       # Project components.
                       PRIV_REQUIRES error)
