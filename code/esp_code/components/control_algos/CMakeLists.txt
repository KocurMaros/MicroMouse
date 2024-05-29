idf_component_register(SRCS "control.c"
                       INCLUDE_DIRS "." 
                                    "include"
									"../Tasks/"
                                    "../motor_driver/include"
                       PRIV_REQUIRES driver nvs_flash esp_timer)
                    