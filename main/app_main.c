#include "es_ble_prov.h"
#include "es_mqtt.h"

#include <nvs_flash.h>
#include <esp_netif.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "cJSON.h"

//static const char *TAG = "APP_MAIN";


#define DEVICE_NAME_MAX_LENGTH 13
static char deviceName[DEVICE_NAME_MAX_LENGTH];
static EdgeSensor edgeSensor;
static ES_Config config;
static ES_StateMachine stateMachine;

typedef struct measurementTaskParams {
    EdgeSensor *edgeSensor;
    esp_mqtt_client_handle_t client;
} MeasurementTaskParams;


void edge_sensor_measurement_task(void *pvParameters) {
    MeasurementTaskParams *params = (MeasurementTaskParams *)pvParameters;
    EdgeSensor *edgeSensor = params->edgeSensor;
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    esp_mqtt_client_handle_t client = params->client;

    while (true) {
        // Wait for semaphore to be given
        printf("Measurement task waiting for signal\n");
        xSemaphoreTake(stateMachine->stateSemaphore, portMAX_DELAY);
        printf("Measurement task received signal\n");
        // Check state under mutex protection

        printf("Measurement task waiting for mutex\n");
        xSemaphoreTake(stateMachine->stateMutex, portMAX_DELAY);
        printf("Measurement task received mutex\n");
        if (stateMachine->state == STATE_WORKING) {
            xSemaphoreGive(stateMachine->stateMutex);
            printf("Measurement task released mutex\n");

            // Edge Sensor takes a measurement
            float measurement = edge_sensor_measure(edgeSensor);

            // Edge Sensor makes a prediction if necessary
            float prediction;
            float *prediction_ptr = NULL;
            if (edgeSensor->config->runPrediction) {
                prediction = edge_sensor_predict(edgeSensor, measurement);
                prediction_ptr = &prediction;
            }

            // Publish measurement to MQTT broker
            publish_measurement(client, edgeSensor->deviceName, measurement, prediction_ptr);

            // Deep sleep
            printf("Measurement task going to sleep\n");
            // edge_sensor_sleep(edgeSensor);

        }
    }
}

void app_init(void) {
    /* Initialize NVS partition */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}


void app_main(void) {
    /* Initialize App */
    app_init();

    /* Initialize BLE Prov*/
    ble_prov_init();
    
    /* Start BLE Prov */
    start_ble_prov();

    /* Get device name from BLE iface MAC address */
    get_device_name(deviceName, DEVICE_NAME_MAX_LENGTH);    

    /* Initialize edge sensor */
    edge_sensor_init(&edgeSensor, &config, &stateMachine, deviceName);

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
