idf_component_register(SRCS "src/config.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES esp_partition
                       # Project components.
                       PRIV_REQUIRES error)
