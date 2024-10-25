#include "es_ble_prov.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "esp_mac.h"

static const char *TAG = "ES_BLE_PROV";

// Signal Wi-Fi events
static SemaphoreHandle_t wifi_semaphore;

// Bluetooth Config
#define PROV_TRANSPORT_BLE "ble"
#define SSID_PREFIX "ESP32_"

// Edge Sensor Partition
#define EDGE_SENSOR_PARTITION "edge_sensor_data"
#define EDGE_SENSOR_PARTITION_NAMESPACE "edge_sensor_ns"
#define POP_KEY "pop_key"
#define EDGE_SENSOR_BUFFER_SIZE 100

esp_err_t get_device_pop(void *pop_data, size_t length)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_blob(my_handle, POP_KEY, pop_data, &length);
    if (err != ESP_OK)
        return err;

    nvs_close(my_handle);

    return ESP_OK;
}

esp_err_t get_device_name(char *device_name, size_t length)
{
    uint8_t ble_mac[6];
    const char *ssid_prefix = SSID_PREFIX;
    esp_err_t err = esp_read_mac(ble_mac, ESP_MAC_BT);
    if (err != ESP_OK)
        return err;
    snprintf(device_name, length, "%s%02X%02X%02X",
             ssid_prefix, ble_mac[3], ble_mac[4], ble_mac[5]);
    return ESP_OK;
}

static void wifi_prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int retries;
    switch (event_id)
    {
    case WIFI_PROV_START:
        ESP_LOGI(TAG, "Provisioning started");
        break;
    case WIFI_PROV_CRED_RECV:
    {
        wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
        ESP_LOGI(TAG, "Received Wi-Fi credentials"
                      "\n\tSSID     : %s\n\tPassword : %s",
                 (const char *)wifi_sta_cfg->ssid,
                 (const char *)wifi_sta_cfg->password);
        break;
    }
    case WIFI_PROV_CRED_FAIL:
    {
        wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
        ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                      "\n\tPlease reset to factory and retry provisioning",
                 (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
        retries++;
        if (retries >= CONFIG_PROV_MGR_MAX_RETRY_CNT)
        {
            ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
            wifi_prov_mgr_reset_sm_state_on_failure();
            retries = 0;
        }
        break;
    }
    case WIFI_PROV_CRED_SUCCESS:
        ESP_LOGI(TAG, "Provisioning successful");
        retries = 0;
        break;
    case WIFI_PROV_END:
        /* De-initialize manager once provisioning is finished */
        // wifi_prov_mgr_deinit();
        break;
    default:
        break;
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        esp_wifi_connect();
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    /* Signal main application to continue execution */
    xSemaphoreGive(wifi_semaphore);
}

static void protocomm_transport_ble_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
        ESP_LOGI(TAG, "BLE transport: Connected!");
        break;
    case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
        ESP_LOGI(TAG, "BLE transport: Disconnected!");
        break;
    default:
        break;
    }
}

static void protocomm_security_session_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
        ESP_LOGI(TAG, "Secured session established!");
        break;
    case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
        ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
        break;
    case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
        ESP_LOGE(TAG, "Received incorrect PoP for establishing secure session!");
        break;
    default:
        break;
    }
}

uint8_t is_device_provisioned(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Reset provisioning [DEBUG]
    //wifi_prov_mgr_reset_provisioning();

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    ESP_ERROR_CHECK(esp_wifi_deinit());

    return provisioned;
}

esp_err_t es_ble_prov_init(void)
{
    // Register our event handler for Wi-Fi, IP and Provisioning related events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &protocomm_transport_ble_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &protocomm_security_session_event_handler, NULL));

    // Initialize Wi-Fi Semaphore
    wifi_semaphore = xSemaphoreCreateBinary();

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg); 

    // Create default Wi-Fi station
    esp_netif_create_default_wifi_sta();

    // Configuration for the provisioning manager
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };

    // Initialize provisioning manager with the configuration parameters set above
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    return ESP_OK;
}

void es_ble_prov_start(void)
{
    /* 
    WIFI_PROV_SECURITY_1: secure handshake using X25519 key exchange
    and proof of possession (pop) and AES-CTR for encryption/decryption of messages. 
    */

    ESP_LOGI(TAG, "Starting provisioning");

    // Get device name
    char device_name[13];
    ESP_ERROR_CHECK(get_device_name(device_name, sizeof(device_name)));
    wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

    // Retrieve the PoP from the NVS partition
    char pop[EDGE_SENSOR_BUFFER_SIZE] = {0}; // Initialize all elements to 0
    size_t pop_length = sizeof(pop) - 1;     // Reserve one byte for null terminator
    ESP_ERROR_CHECK(get_device_pop(pop, pop_length));
    wifi_prov_security1_params_t *sec_params = pop;

    // Set custom service UUID
    uint8_t custom_service_uuid[] = {
        0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
        0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
    };
    wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

    // Start provisioning service
    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)sec_params, device_name, NULL));

    // Wait for Wi-Fi connection
    wifi_prov_mgr_wait();
    wifi_prov_mgr_deinit();
    
    // Wait for Wi-Fi connection
    xSemaphoreTake(wifi_semaphore, portMAX_DELAY);

    // Reboot the device
    esp_restart();
}

esp_err_t es_wifi_init(void)
{
    // Register our event handler for Wi-Fi and IP related events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    // Initialize Wi-Fi Semaphore
    wifi_semaphore = xSemaphoreCreateBinary();

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg); 

    // Create default Wi-Fi station
    esp_netif_create_default_wifi_sta();

    return ESP_OK;
}

void es_wifi_start(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Wait for Wi-Fi connection */
    xSemaphoreTake(wifi_semaphore, portMAX_DELAY);
}
