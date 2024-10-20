#include <esp_err.h>
#include <string.h>

esp_err_t get_device_pop(void *pop_data, size_t length);

esp_err_t get_device_name(char *device_name, size_t length);

uint8_t is_device_provisioned(void);

esp_err_t es_ble_prov_init(void);

void es_ble_prov_start(void);

esp_err_t es_wifi_init(void);

void es_wifi_start(void);


