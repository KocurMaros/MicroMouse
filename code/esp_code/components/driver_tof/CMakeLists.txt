idf_component_register(SRCS "tof.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash esp_timer)
