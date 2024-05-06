idf_component_register(SRCS "task_control.cpp" "task_meas.cpp" "../ahrs/ahrs.c" "../mpu9250/mpu9250.c" "../mpu9250/ak8963.c" "../mpu9250/calibrate.c" "../mpu9250/common.c" "../mpu9250/i2c-easy.c" "../motor_driver/driver.c"
                       INCLUDE_DIRS "."
                       "../driver_tof"
                       "../driver_tof/core/inc"
                       "../driver_tof/platform/inc"
                       "../ahrs/include/" 
                       "../mpu9250/include/"
                       "../motor_driver/include/"
                       PRIV_REQUIRES driver nvs_flash esp_timer)
                       target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)