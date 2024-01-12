idf_component_register(SRCS "atmel.c"
		REQUIRES driver nvs_flash
                       INCLUDE_DIRS "."
                       )
