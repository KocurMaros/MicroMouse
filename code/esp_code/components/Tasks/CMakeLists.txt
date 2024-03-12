idf_component_register(SRCS "task_acquire.c" "task_communication.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash esp_timer)
