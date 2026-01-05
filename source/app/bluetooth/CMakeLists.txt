idf_component_register(SRCS "src/bluetooth.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES esp_hid
                       PRIV_REQUIRES esp_pm
                       # Project components.
                       PRIV_REQUIRES error
                       PRIV_REQUIRES led)
