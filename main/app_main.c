#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_wifi.h>

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
#define POLL_WAIT_TIME 500

static const char *TAG = "APP_MAIN";

// Edge Sensor
static EdgeSensor edgeSensor;
static char deviceName[DEVICE_NAME_MAX_LENGTH];

// Measurement Task
typedef struct _measurementTaskParams
{
    EdgeSensor *edgeSensor;
} MeasurementTaskParams;

static MeasurementTaskParams measurementTaskParams;
static TaskHandle_t measurementTaskHandle;

// MQTT
static MqttHandlerArgs mqttHandlerArgs;
static esp_mqtt_client_handle_t client;

// RTC Data
typedef struct
{
    uint8_t sleep_flag;
    uint8_t poll_period;
    uint8_t poll_idx;
} rtc_data_t;
RTC_DATA_ATTR static rtc_data_t rtc_data = {.sleep_flag = 0, .poll_period = 20, .poll_idx = 0};

// MQTT Client
void launch_mqtt_client(){
    // Initialize Wi-Fi
    ESP_LOGI(TAG, "Initializing Wi-Fi");
    ESP_ERROR_CHECK(es_wifi_init());

    // Start Wi-Fi
    ESP_LOGI(TAG, "Starting Wi-Fi");
    es_wifi_start();

    // Initialize MQTT
    ESP_LOGI(TAG, "Initializing MQTT");
    ESP_ERROR_CHECK(es_mqtt_init(&client));

    // Start MQTT
    ESP_LOGI(TAG, "Starting MQTT");
    mqttHandlerArgs.edgeSensor = &edgeSensor;
    mqttHandlerArgs.measurementTaskHandle = measurementTaskHandle;
    es_mqtt_start(&client, &mqttHandlerArgs);
    
    // Wait for MQTT to connect and subscribe to topics
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

// Measurement Task
void __handle_on_device_inf(EdgeSensor *edgeSensor)
{
    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Get sensor reading buffer and predictive model
    float32_t **readingBuffer = edgeSensor->bmi270->readingBuffer;
    ES_PredictiveModel *predictiveModel = edgeSensor->predictiveModel;

    // Measure reading from bmi270
    es_bmi270_measure(readingBuffer);

    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // DEBUG: copy the first 25 samples to the model input buffer
    float **buff = malloc(25 * sizeof(float *));
    for (int i = 0; i < 25; i++)
    {
        buff[i] = malloc(6 * sizeof(float));
        for (int j = 0; j < 6; j++)
        {
            buff[i][j] = readingBuffer[i][j];
        }
    }

    // Initialize the TFLM model
    es_tflite_init(predictiveModel->buffer);

    // Perform on-device inference
    uint8_t tf_model_output;
    es_tflite_predict(buff, &tf_model_output);

    // Free reading buffer
    free_reading_buffer(readingBuffer);
    free(buff);

    // Update Prediction Counter and History
    UPDATE_PREDICTION_COUNTER(predictiveModel->predictionCounter);
    UPDATE_PREDICTION_HISTORY(predictiveModel->predictionHistory, tf_model_output);
    edge_sensor_update_inference_layer(edgeSensor);

    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Poll Enqueued Commands if poll index is 0
    if (rtc_data.poll_idx == 0)
    {
        // Launch MQTT Client
        launch_mqtt_client();

        // Wait a fixed amount of time to ensure all commands are received
        vTaskDelay(POLL_WAIT_TIME / portTICK_PERIOD_MS);
    }
}

void __handle_offload_inf(EdgeSensor *edgeSensor)
{
    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Get sensor raw reading buffer
    uint8_t *rawReadingBuffer = edgeSensor->bmi270->rawReadingBuffer;

    // Measure raw reading from bmi270
    es_bmi270_raw_measure(rawReadingBuffer);

    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Compress raw reading
    size_t compressed_length;
    uint8_t *compressed_reading = compress_bytes(rawReadingBuffer, BMI270_SEQUENCE_LENGTH * BMI270_SAMPLE_SIZE * 2, &compressed_length);
    free_raw_reading_buffer(rawReadingBuffer);

    // Encode compressed reading to base64
    size_t b64_encoded_length;
    char *b64_gzip_reading = encode_base64(compressed_reading, compressed_length, &b64_encoded_length);
    free(compressed_reading);

    // DEBUG: delay for 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Create cJSON object
    cJSON *jsonPayload = cJSON_CreateObject();

    // Build export sensor data payload
    SensorDataExport sensorDataExport = {
        .lowBattery = IS_LOW_BATTERY(edgeSensor->remainingBattery),
        .inferenceLayer = edgeSensor->inferenceLayer,
        .b64_gzip_reading = b64_gzip_reading,
        .sendTimestamp = NULL,
        .recvTimestamp = NULL,
        .prediction = NULL
    };
    build_export_sensor_data_payload(jsonPayload, &sensorDataExport);

    // Convert cJSON object to payload
    char *payload = cJSON_Print(jsonPayload);

    // Free cJSON object
    cJSON_Delete(jsonPayload);

    // Free b64_gzip_reading
    free(b64_gzip_reading);

    // Launch MQTT Client
    launch_mqtt_client();

    // Publish sensor data
    publish_export_sensor_data(client, edgeSensor->deviceName, payload);

    // Wait a fixed amount of time to ensure all commands are received
    vTaskDelay(POLL_WAIT_TIME / portTICK_PERIOD_MS);

    // DEBUG: delay for 5 seconds
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
}

void edge_sensor_measurement_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting Edge Sensor Measurement Task");
    MeasurementTaskParams *params = (MeasurementTaskParams *)pvParameters;
    EdgeSensor *edgeSensor = params->edgeSensor;
    ES_State state = edgeSensor->stateMachine->state;
    while (true)
    {
        switch (state)
        {
        case STATE_WORKING:
            ESP_LOGI(TAG, "Edge Sensor State: STATE_WORKING.\n");
            ES_InferenceLayer inferenceLayer = edgeSensor->inferenceLayer;
            if (inferenceLayer == INFERENCE_LAYER_SENSOR)
            {
                // Perform on-device inference
                __handle_on_device_inf(edgeSensor);
            }
            else
            {
                // Perform offload inference
                __handle_offload_inf(edgeSensor);
            }
            break;

        default:
            ESP_LOGI(TAG, "Edge Sensor State: %s.\n", FROM_ENUM_STATE_TO_STRING(state));

            // Launch MQTT Client
            launch_mqtt_client();

            // Wait a fixed amount of time to ensure all commands are received
            vTaskDelay(POLL_WAIT_TIME / portTICK_PERIOD_MS);
            break;
        }

        /* Save edge sensor to NVS */
        ESP_LOGI(TAG, "Saving edge sensor to NVS.\n");
        ESP_ERROR_CHECK(save_edge_sensor_to_nvs(edgeSensor));

        /* Sleep edge sensor */
        ESP_LOGI(TAG, "Sleeping edge sensor.\n");
        edge_sensor_sleep(edgeSensor);
    }
}


// Main App
esp_err_t app_init(void)
{
    // Update the poll index
    ESP_LOGI(TAG, "Updating poll index");
    rtc_data.poll_idx = (rtc_data.poll_idx + 1) % rtc_data.poll_period;

    // Initialize event loop
    ESP_LOGI(TAG, "Initializing event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS partition
    ESP_LOGI(TAG, "Initializing NVS partition");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(EDGE_SENSOR_PARTITION));

    // Init Wi-Fi and check if provisioned
    if (!is_device_provisioned())
    {
        // Initialize BLE Prov
        ESP_LOGI(TAG, "Initializing BLE Prov");
        ESP_ERROR_CHECK(es_ble_prov_init());

        // Start BLE Prov
        ESP_LOGI(TAG, "Starting BLE Prov");
        es_ble_prov_start(); // Reboots after provisioning
    }

    // Initialize the sensor
    ESP_LOGI(TAG, "Initializing the sensor");
    get_device_name(deviceName, DEVICE_NAME_MAX_LENGTH);
    ESP_LOGI(TAG, "Device Name: %s\n", deviceName);

    ESP_LOGI(TAG, "Initializing Edge Sensor Struct.\n");
    ESP_ERROR_CHECK(edge_sensor_init(&edgeSensor, deviceName, rtc_data.sleep_flag));

    // Initialize the sensor
    ES_BMI270 *bmi270 = edgeSensor.bmi270;
    es_bmi270_init(bmi270->configSize, bmi270->configBuffer, rtc_data.sleep_flag);

    // Update sleep flag
    rtc_data.sleep_flag = 1;

    return ESP_OK;
}

void app_main(void)
{
    // step 0: Initialize the app
    ESP_LOGI(TAG, "Initializing the APP");
    ESP_ERROR_CHECK(app_init());

    // step 1: Set up the measurement task parameters
    measurementTaskParams.edgeSensor = &edgeSensor;

    // step 2: Create the measurement task
    xTaskCreate(edge_sensor_measurement_task, "edge_sensor_measurement_task", 4096, &measurementTaskParams, 5, &measurementTaskHandle);
    if (measurementTaskHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create edge sensor measurement task");
        vTaskDelete(measurementTaskHandle);
    }
}