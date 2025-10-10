idf_component_register(SRCS "src/push-button.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES driver
                       # Project components.
                       PRIV_REQUIRES error
                       PRIV_REQUIRES led
                       PRIV_REQUIRES power-latch)
