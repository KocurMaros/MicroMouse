idf_component_register(SRCS "driver.c"
                       INCLUDE_DIRS "." 
                                    "include"
									"../Tasks/"
                       PRIV_REQUIRES driver nvs_flash esp_timer)
                    