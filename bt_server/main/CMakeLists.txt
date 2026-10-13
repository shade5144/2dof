idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES bt_manager adc_manager mcpwm_manager nvs_flash
                    INCLUDE_DIRS ".")
