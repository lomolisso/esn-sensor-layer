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
#include "esp_timer.h"

// static const char *TAG = "APP_MAIN";

#define DEVICE_NAME_MAX_LENGTH 13

static EdgeSensor edgeSensor;
static char deviceName[DEVICE_NAME_MAX_LENGTH];

typedef struct measurementTaskParams
{
    EdgeSensor *edgeSensor;
    esp_mqtt_client_handle_t client;
} MeasurementTaskParams;

/* Utils */
void printHeapInfo() {
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    printf("Free heap size: %zu bytes\n", freeHeap);
    size_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    printf("Largest free contiguous heap block: %zu bytes\n", largestFreeBlock);
}

void from_float_to_string(float value, char *buffer)
{
    sprintf(buffer, "%f", value);
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
            /* Compute inference layers */
            uint8_t predLayers = edge_sensor_compute_inference_layers(edgeSensor);
            printf("Prediction on Cloud: %d\n", PREDICTION_ON_CLOUD(predLayers));
            printf("Prediction on Gateway: %d\n", PREDICTION_ON_GATEWAY(predLayers));
            printf("Prediction on Device: %d\n", PREDICTION_ON_DEVICE(predLayers));

            /* Edge Sensor takes a measurement */
            float measurement = edge_sensor_measure(edgeSensor);

            /* Generate JSON log payload */
            cJSON *jsonPayload = cJSON_CreateObject();
            cJSON_AddNumberToObject(jsonPayload, "measurement", measurement);

            /* Publish measurement to MQTT broker */
            publish_measurement(client, edgeSensor->deviceName, jsonPayload);

            /* Free the cJSON object */
            cJSON_Delete(jsonPayload);

            if (PREDICTION_ON_GATEWAY(predLayers) || PREDICTION_ON_CLOUD(predLayers)) {
                /* Generate JSON log payload */
                cJSON *jsonPayload = cJSON_CreateObject();

                /* Get a timestamp from the timer */
                uint64_t _uint64_timestamp = edge_sensor_timer_get_time();
                printf("Timestamp in microseconds: %llu (us)\n", _uint64_timestamp);
                char request_timestamp[21] = {0};
                from_uint64_to_string(_uint64_timestamp, request_timestamp);

                /* generate prediction request payload */
                edge_sensor_prediction_request_payload(jsonPayload, request_timestamp, measurement);

                /* publish prediction request to MQTT broker */
                publish_prediction_request(client, edgeSensor->deviceName, jsonPayload, predLayers);

                /* Free the cJSON object */
                cJSON_Delete(jsonPayload);
            }


            if (PREDICTION_ON_DEVICE(predLayers)) {
                float prediction;
                es_tflite_predict(&measurement, &prediction);
                
                // TODO: implement handle for prediction
                // [...]

                /* Generate JSON log payload */
                if (ESN_DEBUG_MODE) {
                    printf("Debug mode: generating JSON log payload\n");
                    cJSON *jsonPayload = cJSON_CreateObject();

                    char measurement_buffer[10] = {0};
                    from_float_to_string(measurement, measurement_buffer);
                    char prediction_buffer[10] = {0};
                    from_float_to_string(prediction, prediction_buffer);
                    edge_sensor_prediction_log_payload(jsonPayload, "edge-sensor", "0", "0", measurement_buffer, prediction_buffer);
                    publish_prediction_log(client, edgeSensor->deviceName, jsonPayload);
                    cJSON_Delete(jsonPayload);
                }
            }
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

    uint64_t timestamp = edge_sensor_timer_get_time(); // microseconds
    printf("Timestamp in miliseconds: %llu (ms)\n", timestamp / 1000);
}