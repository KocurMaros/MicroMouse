idf_component_register(SRCS "task_control.c" "task_meas.c"
                       INCLUDE_DIRS "."
                       "../driver_tof"
                       "../driver_tof/core/inc"
                       "../driver_tof/platform/inc"
                       PRIV_REQUIRES driver nvs_flash esp_timer)
