#include <nvs_flash.h>
#include <esp_netif.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "es_ble_prov.h"
#include "es_mqtt.h"
#include "es_tflite.h"
#include "es_bmi270.h"

#define DEVICE_NAME_MAX_LENGTH 13
#define EDGE_SENSOR_PARTITION "edge_sensor_data"

typedef struct _measurementTaskParams
{
    EdgeSensor *edgeSensor;
    esp_mqtt_client_handle_t client;
} MeasurementTaskParams;


static const char *TAG = "APP_MAIN";
static EdgeSensor edgeSensor;
static char deviceName[DEVICE_NAME_MAX_LENGTH];

static MeasurementTaskParams measurementTaskParams;
static TaskHandle_t measurementTaskHandle;

static MqttHandlerArgs mqttHandlerArgs;
static esp_mqtt_client_handle_t client;

typedef struct {
    uint8_t sleep_flag;
} rtc_data_t;

RTC_DATA_ATTR static rtc_data_t rtc_data = { .sleep_flag = 0 };

/* Utils */
void printHeapInfo()
{
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Free heap size: %zu bytes\n", freeHeap);
    size_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Largest free contiguous heap block: %zu bytes\n", largestFreeBlock);
}

/* Measurement Task */
void edge_sensor_measurement_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting Edge Sensor Measurement Task");
    MeasurementTaskParams *params = (MeasurementTaskParams *)pvParameters;
    EdgeSensor *edgeSensor = params->edgeSensor;
    //esp_mqtt_client_handle_t client = params->client;

    SemaphoreHandle_t mutexSemaphore = edgeSensor->mutexSemaphore;
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    ES_PredictiveModel *predictiveModel = edgeSensor->predictiveModel;
    float32_t **readingBuffer = edgeSensor->bmi270->readingBuffer;

    while (true)
    {
        /* Wait for the mqtt handler to notify the task */
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Retrieve Edge Sensor State Safely */
        ESP_LOGI(TAG, "Mutex Semaphore requested.\n");
        xSemaphoreTake(mutexSemaphore, portMAX_DELAY);
        ESP_LOGI(TAG, "Mutex Semaphore received.\n");
        ES_State state = stateMachine->state;
        ESP_LOGI(TAG, "Mutex Semaphore given.\n");
        xSemaphoreGive(mutexSemaphore);

        switch (state)
        {
        case STATE_WORKING:
            ESP_LOGI(TAG, "Edge Sensor State: STATE_WORKING.\n");
            cJSON *jsonPayload = cJSON_CreateObject();
            /* Get sensor reading */

            vTaskDelay(2000 / portTICK_PERIOD_MS);

            ESP_LOGI(TAG, "Measuring BMI270 Sensor.\n");
            es_bmi270_measure(readingBuffer);

            /* Retrieve Edge Sensor Inference Layer Safely */
            ESP_LOGI(TAG, "Mutex Semaphore requested.\n");
            xSemaphoreTake(mutexSemaphore, portMAX_DELAY);
            ESP_LOGI(TAG, "Mutex Semaphore received.\n");
            ESP_LOGI(TAG, "Mutex Semaphore given.\n");
            xSemaphoreGive(mutexSemaphore);

            /* Perform on-device inference if necessary */
            uint8_t *prediction = NULL;
            uint8_t tf_model_output;
            
            if (edgeSensor->inferenceLayer == INFERENCE_LAYER_SENSOR)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                // Initialize TFLM model
                ESP_LOGI(TAG, "Initializing TFLM model.\n");
                es_tflite_init(predictiveModel->buffer);
                
                // Run TFLite model
                ESP_LOGI(TAG, "Performing on-device inference.\n");
                
                es_tflite_predict(readingBuffer, &tf_model_output);
        
                UPDATE_PREDICTION_COUNTER(predictiveModel->predictionCounter);
                UPDATE_PREDICTION_HISTORY(predictiveModel->predictionHistory, tf_model_output);
                edge_sensor_update_inference_layer(edgeSensor);
                prediction = &tf_model_output;
                
                 vTaskDelay(2000 / portTICK_PERIOD_MS);
            }


            /* Build JSON payload */
            ESP_LOGI(TAG, "Building export 'sensor-data' JSON payload.\n");
            SensorDataExport sensorDataExport = {
                .lowBattery = IS_LOW_BATTERY(edgeSensor->remainingBattery),
                //.inferenceLayer = edgeSensor->inferenceLayer,
                .inferenceLayer = INFERENCE_LAYER_GATEWAY,
                .reading = readingBuffer,
                .sendTimestamp = NULL,
                .recvTimestamp = NULL,
                .prediction = prediction};
            build_export_sensor_data_payload(jsonPayload, &sensorDataExport);

            /* Publish sensor data */
            //ESP_LOGI(TAG, "Publishing export 'sensor-data' to MQTT broker.\n");
            //publish_export_sensor_data(client, edgeSensor->deviceName, jsonPayload);

            /* Free JSON payload */
            cJSON_Delete(jsonPayload);
            break;

        default:
            ESP_LOGI(TAG, "Edge Sensor State: %s.\n", FROM_ENUM_STATE_TO_STRING(stateMachine->state));
            break;
        }

        /* Take Mutex Semaphore */
        ESP_LOGI(TAG, "Mutex Semaphore requested.\n");
        xSemaphoreTake(mutexSemaphore, portMAX_DELAY);
        ESP_LOGI(TAG, "Mutex Semaphore received.\n");

        /* Save edge sensor to NVS */
        ESP_LOGI(TAG, "Saving edge sensor to NVS.\n");
        ESP_ERROR_CHECK(save_edge_sensor_to_nvs(edgeSensor));

        /* Sleep edge sensor */
        ESP_LOGI(TAG, "Sleeping edge sensor.\n");
        edge_sensor_sleep(edgeSensor);
    }
}

void app_init(void)
{
    /* Initialize NVS partition */
    ESP_LOGI(TAG, "Initializing NVS partition");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(EDGE_SENSOR_PARTITION));

    /* Initialize TCP/IP */
    //ESP_LOGI(TAG, "Initializing TCP/IP stack");
    //ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize event loop */
    ESP_LOGI(TAG, "Initializing event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize BLE Prov*/
    //ESP_LOGI(TAG, "Initializing BLE Prov");
    //ESP_ERROR_CHECK(ble_prov_init());
}

void app_main(void)
{
    /* Initialize App */
    ESP_LOGI(TAG, "Initializing App");
    app_init();

    /* Start BLE Prov */
    //ESP_LOGI(TAG, "Starting BLE Prov");
    //start_ble_prov();

    /* Get device name from BLE iface MAC address */
    //get_device_name(deviceName, DEVICE_NAME_MAX_LENGTH);
    char *deviceName = "ESP32_24A1C2";
    ESP_LOGI(TAG, "Device Name: %s\n", deviceName);

    //ESP_ERROR_CHECK(reset_nvs_edge_sensor());
    ESP_LOGI(TAG, "Initializing Edge Sensor Struct.\n");
    printHeapInfo();
    ESP_ERROR_CHECK(edge_sensor_init(&edgeSensor, deviceName));
    ESP_LOGI(TAG, "Edge sensor initialized.\n");
    printHeapInfo();

    /* Initialize BMI270 Sensor */
    ES_BMI270 *bmi270 = edgeSensor.bmi270;
    if (bmi270 != NULL)
    {
        ESP_LOGI(TAG, "Initializing BMI270 Sensor.\n");
        es_bmi270_init(bmi270->configSize, bmi270->configBuffer, rtc_data.sleep_flag);
    }
    else
    {
        ESP_LOGI(TAG, "BMI270 sensor is NULL, BMI270 initialization skipped.\n");
    }

    rtc_data.sleep_flag = 1;

    /* Initialize TFLM model */
    //ES_PredictiveModel *predictiveModel = edgeSensor.predictiveModel;
    //edgeSensor.inferenceLayer = INFERENCE_LAYER_SENSOR;
    //if (predictiveModel != NULL && edgeSensor.inferenceLayer == INFERENCE_LAYER_SENSOR)
    //{
    //    ESP_LOGI(TAG, "Initializing TFLM model.\n");
    //   es_tflite_init(predictiveModel->buffer);
    //}
    //else
    //{
    //    ESP_LOGI(TAG, "Predictive model is NULL or inference layer is not INFERENCE_LAYER_SENSOR, TFLM initialization skipped.\n");
    //}

    /* Initialize MQTT */
    //ESP_LOGI(TAG, "Initializing MQTT Client");
    //client = mqtt_init();

    /* Create Edge Sensor Measurement Task */
    measurementTaskParams.edgeSensor = &edgeSensor;
    measurementTaskParams.client = client;
    
    xTaskCreate(edge_sensor_measurement_task, "edge_sensor_measurement_task", 4096, &measurementTaskParams, 5, &measurementTaskHandle);
    if (measurementTaskHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create edge sensor measurement task");
        vTaskDelete(measurementTaskHandle);
    }

    /* Start MQTT */
    //ESP_LOGI(TAG, "Starting MQTT Client");
    //mqttHandlerArgs.edgeSensor = &edgeSensor;
    //mqttHandlerArgs.measurementTaskHandle = measurementTaskHandle;
    //start_mqtt(client, &mqttHandlerArgs);
}