idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                                 "../components/udp/include")
                    target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)