idf_component_register(SRCS "task_control.c" "task_meas.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash esp_timer)
