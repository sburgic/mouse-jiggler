idf_component_register(SRCS "src/led.c"
                       INCLUDE_DIRS "api"
                       # ESP-IDF components.
                       PRIV_REQUIRES driver
                       # Project components.
                       PRIV_REQUIRES error)
