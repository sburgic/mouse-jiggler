idf_component_register(SRCS "src/bluetooth.c"
                            "src/esp_hid_gap.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES esp_hid
                       # Project components.
                       PRIV_REQUIRES error
                       PRIV_REQUIRES led)
