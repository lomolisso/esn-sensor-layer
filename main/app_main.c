#include "es_ble_prov.h"
#include "es_mqtt.h"

#include <nvs_flash.h>
#include <esp_netif.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "es_tflite.h"
#include "cJSON.h"

#include "esp_heap_caps.h"


// static const char *TAG = "APP_MAIN";

#define DEVICE_NAME_MAX_LENGTH 13

static EdgeSensor edgeSensor;
static char deviceName[DEVICE_NAME_MAX_LENGTH];

typedef struct measurementTaskParams
{
    EdgeSensor *edgeSensor;
    esp_mqtt_client_handle_t client;
} MeasurementTaskParams;

void printHeapInfo() {
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    printf("Free heap size: %zu bytes\n", freeHeap);
    size_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    printf("Largest free contiguous heap block: %zu bytes\n", largestFreeBlock);
}

void edge_sensor_measurement_task(void *pvParameters)
{
    MeasurementTaskParams *params = (MeasurementTaskParams *)pvParameters;
    EdgeSensor *edgeSensor = params->edgeSensor;
    SemaphoreHandle_t commandSemaphore = edgeSensor->commandSemaphore;
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    esp_mqtt_client_handle_t client = params->client;

    while (true)
    {   
        /* Wait for command signal */
        printf("Measurement task waiting for command signal\n");
        xSemaphoreTake(commandSemaphore, portMAX_DELAY);
        printf("Measurement task received command signal\n");

        /* Check state and perform measurement */
        if (stateMachine->state == STATE_WORKING) {
            // Edge Sensor takes a measurement
            float measurement = edge_sensor_measure(edgeSensor);

            // Edge Sensor makes a prediction if necessary
            float prediction;
            float *prediction_ptr = NULL;

            if (edgeSensor->predictiveModel != NULL) {
                es_tflite_predict(&measurement, &prediction);
                prediction_ptr = &prediction;
            }

            // Publish measurement to MQTT broker
            publish_measurement(client, edgeSensor->deviceName, measurement, prediction_ptr);
        }
        
        /* Save edge sensor to NVS */
        ESP_ERROR_CHECK(save_edge_sensor_to_nvs(edgeSensor));
        
        printf("Measurement task called edge_sensor_sleep\n");
        edge_sensor_sleep(edgeSensor);
    }
}

void app_init(void)
{
    /* Initialize NVS partition */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize BLE Prov*/
    ble_prov_init();
}

void app_main(void)
{
    /* Initialize App */
    app_init();

    /* Start BLE Prov */
    start_ble_prov();

    /* Get device name from BLE iface MAC address */
    get_device_name(deviceName, DEVICE_NAME_MAX_LENGTH);

    //ESP_ERROR_CHECK(reset_nvs_edge_sensor());
    uint8_t sleep_flag = get_nvs_edge_sensor_sleep_flag();
    uint8_t model_flag = get_nvs_edge_sensor_model_flag();
    uint8_t config_flag = get_nvs_edge_sensor_config_flag();
    printHeapInfo();
    ESP_ERROR_CHECK(edge_sensor_init(&edgeSensor, deviceName, sleep_flag, model_flag, config_flag));
    printf("Edge sensor initialized\n");
    printHeapInfo();
    es_tflite_init(edgeSensor.predictiveModel);

    /* Initialize MQTT */
    esp_mqtt_client_handle_t client = mqtt_init();

    /* Launch edge sensor measurement task */
    MeasurementTaskParams params = {
        .edgeSensor = &edgeSensor,
        .client = client,
    };
    xTaskCreate(edge_sensor_measurement_task, "edge_sensor_measurement_task", 4096, &params, 5, NULL);

    /* Start MQTT */
    start_mqtt(client, &edgeSensor);
}