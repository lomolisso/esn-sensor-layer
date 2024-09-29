idf_component_register(SRCS "edge_sensor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "freertos" "nvs_flash" "json" "esp_hw_support" "mbedtls" "esp_timer" "es_miniz"
)
