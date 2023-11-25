idf_component_register(
                SRCS "es_ble_prov.c"
                INCLUDE_DIRS "include"
                REQUIRES "esp_wifi" "nvs_flash" "wifi_provisioning"
)
