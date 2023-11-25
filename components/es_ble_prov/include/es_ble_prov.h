#include <esp_err.h>
#include <string.h>

esp_err_t get_device_pop(void *pop_data, size_t length);

esp_err_t get_device_name(char *device_name, size_t length);

void ble_prov_init();

void start_ble_prov(void);
