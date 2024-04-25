idf_component_register(SRCS "udp_server.c"
                    INCLUDE_DIRS "."
                                 "../protocol_examples_common/include"
                    PRIV_REQUIRES nvs_flash protocol_examples_common)