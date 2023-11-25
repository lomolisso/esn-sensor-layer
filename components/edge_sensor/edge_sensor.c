#include "edge_sensor.h"

#include <stdio.h>  // for printf()
#include <stdlib.h> // for rand()
#include <time.h>   // for time()

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "cJSON.h"
#include <string.h>

#include "esp_sleep.h"

#define EDGE_SENSOR_PARTITION "edge_sensor_data"
#define EDGE_SENSOR_PARTITION_NAMESPACE "edge_sensor_ns"
#define DEVICE_NAME_MAX_LENGTH 13

/* Utils */
bool starts_with(const char *pre, const char *str) {
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

/* Random number generator */
void init_random()
{
    srand((unsigned)time(NULL));
}

float get_random_float32()
{
    return (float)rand() / (float)RAND_MAX;
}

/* Parsing */
bool validate_json(cJSON *jsonPayload)
{
    if (jsonPayload == NULL)
    {
        const char *error = cJSON_GetErrorPtr();
        if (error != NULL)
        {
            fprintf(stderr, "Error before: %s\n", error);
        }
    }
    return jsonPayload != NULL;
}


/* NVS */
esp_err_t save_edge_sensor_to_nvs(EdgeSensor *edgeSensor)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    // step 1: save fields from EdgeSensor and member structs
    if (edgeSensor->deviceName)
    {
        err = nvs_set_str(nvs_handle, "deviceName", edgeSensor->deviceName);
        if (err != ESP_OK)
            return err;
    }

    if (edgeSensor->config)
    {
        err = nvs_set_i32(nvs_handle, "measurementIntervalMS", edgeSensor->config->measurementIntervalMS);
        if (err != ESP_OK)
            return err;

        err = nvs_set_u8(nvs_handle, "runPrediction", edgeSensor->config->runPrediction);
        if (err != ESP_OK)
            return err;

        err = nvs_set_str(nvs_handle, "predictiveModel", edgeSensor->config->predictiveModel);
        if (err != ESP_OK)
            return err;
    }

    // step 2: save a boolean indicating that the edge sensor has been configured
    err = nvs_set_u8(nvs_handle, "configured", 1);
    if (err != ESP_OK)
        return err;

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
        return err;

    // Close NVS
    nvs_close(nvs_handle);

    return ESP_OK;
}

esp_err_t check_edge_sensor_configured(uint8_t *configured)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_u8(nvs_handle, "configured", configured);
    if (err != ESP_OK)
        return err;

    nvs_close(nvs_handle);

    return ESP_OK;
}

esp_err_t load_edge_sensor_from_nvs(EdgeSensor *edgeSensor, ES_Config *config, ES_StateMachine *stateMachine, char *deviceName)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;

    // step 1: initialize edge sensor
    edgeSensor->deviceName = deviceName;
    edgeSensor->commandHandler = es_command_handler;
    edgeSensor->config = config;
    edgeSensor->stateMachine = stateMachine;

    // step 2: retrieve device name from NVS
    size_t required_size;
    err = nvs_get_str(nvs_handle, "deviceName", NULL, &required_size);
    if (err != ESP_OK)
        return err;

    err = nvs_get_str(nvs_handle, "deviceName", edgeSensor->deviceName, &required_size);
    if (err != ESP_OK)
        return err;

    // step 2: re-initialize state machine (loading from NVS means the device
    // is waking up from deep sleep and resuming working state).
    stateMachine->state = STATE_WORKING;
    stateMachine->stateHandler = state_working_handler;
    stateMachine->stateMutex = xSemaphoreCreateMutex();
    stateMachine->stateSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(stateMachine->stateSemaphore); // avoid waiting for semaphore in measurement task.

    // step 3: initialize config from NVS
    err = nvs_get_u32(nvs_handle, "measurementIntervalMS", &edgeSensor->config->measurementIntervalMS);
    if (err != ESP_OK)
        return err;

    err = nvs_get_u8(nvs_handle, "runPrediction", &edgeSensor->config->runPrediction);
    if (err != ESP_OK)
        return err;

    err = nvs_get_str(nvs_handle, "predictiveModel", NULL, &required_size);
    if (err != ESP_OK)
        return err;

    edgeSensor->config->predictiveModel = malloc(required_size);
    err = nvs_get_str(nvs_handle, "predictiveModel", edgeSensor->config->predictiveModel, &required_size);
    if (err != ESP_OK)
        return err;

    // Close NVS
    nvs_close(nvs_handle);

    return ESP_OK;
}

/* Edge Sensor Config */
void es_config_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonConfigField)
{
    if (strcmp(commandName, "config-measurement-interval-ms") == 0) {
        cJSON *JSON_measurementIntervalMS = cJSON_GetObjectItem(jsonConfigField, commandName);
        edgeSensor->config->measurementIntervalMS = cJSON_IsNumber(JSON_measurementIntervalMS) ? JSON_measurementIntervalMS->valueint : 1000;

    } else if (strcmp(commandName, "config-run-prediction") == 0) {
        cJSON *JSON_runPrediction = cJSON_GetObjectItem(jsonConfigField, commandName);
        edgeSensor->config->runPrediction = cJSON_IsTrue(JSON_runPrediction);

    } else if (strcmp(commandName, "config-predictive-model") == 0) {
        cJSON *JSON_predictiveModel = cJSON_GetObjectItem(jsonConfigField, commandName);
        size_t required_size = strlen(JSON_predictiveModel->valuestring) + 1;
        edgeSensor->config->predictiveModel = (char *) malloc(required_size);
        strcpy(edgeSensor->config->predictiveModel, JSON_predictiveModel->valuestring);


    } else if (strcmp(commandName, "config-done") == 0) {
        /* handle event CONFIG_COMMAND_RECEIVED */
        printf("Edge Sensor taking mutex\n");
        xSemaphoreTake(edgeSensor->stateMachine->stateMutex, portMAX_DELAY);
        edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_CONFIG_COMMAND_RECEIVED);
        xSemaphoreGive(edgeSensor->stateMachine->stateMutex);
        printf("Edge Sensor released mutex\n");

    } else {
        printf("Config command '%s' not recognized.\n", commandName);
    }
}

/* Edge Sensor Command Handler */
void es_command_handler(EdgeSensor *edgeSensor, const char *commandName, const char *commandMethod, const char *payload)
{
    if (strcmp(commandMethod, "set") == 0)
    {
        /* parse payload into JSON */
        cJSON *jsonPayload = cJSON_Parse(payload);
        if (!validate_json(jsonPayload))
        {
            printf("Invalid JSON payload: %s\n", payload);
            return;
        }
        /* handle command */
        if (starts_with("config", commandName))
        {
            es_config_command_handler(edgeSensor, commandName, jsonPayload);
        }
        else if (strcmp(commandName, "edge-sensor-working-status") == 0)
        {
            cJSON *workingStatus = cJSON_GetObjectItem(jsonPayload, commandName);
            if (cJSON_IsTrue(workingStatus))
            {
                /* handle event START_COMMAND_RECEIVED */
                printf("Edge Sensor taking mutex\n");
                xSemaphoreTake(edgeSensor->stateMachine->stateMutex, portMAX_DELAY);
                edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_START_COMMAND_RECEIVED);
                xSemaphoreGive(edgeSensor->stateMachine->stateMutex);
                printf("Edge Sensor released mutex\n");
            }
            else
            {
                /* handle event STOP_COMMAND_RECEIVED */
                printf("Edge Sensor taking mutex\n");
                xSemaphoreTake(edgeSensor->stateMachine->stateMutex, portMAX_DELAY);
                edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_STOP_COMMAND_RECEIVED);
                xSemaphoreGive(edgeSensor->stateMachine->stateMutex);
                printf("Edge Sensor released mutex\n");
            }
        }
        else if (strcmp(commandName, "edge-sensor-reset") == 0)
        {
            /* handle event RESET_COMMAND_RECEIVED */
            printf("Edge Sensor taking mutex\n");
            xSemaphoreTake(edgeSensor->stateMachine->stateMutex, portMAX_DELAY);
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_RESET_COMMAND_RECEIVED);
            xSemaphoreGive(edgeSensor->stateMachine->stateMutex);
            printf("Edge Sensor released mutex\n");
        }
        else
        {
            printf("Command '%s' not recognized.\n", commandName);
        }
    }
    else if (strcmp(commandMethod, "get") == 0)
    {
        // TODO: implement get command
    }
    else
    {
        printf("Command method '%s' not recognized.\n", commandMethod);
    }
}

/* State Machine handlers*/
void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_CONFIG_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_CONFIGURED;
        edgeSensor->stateMachine->stateHandler = state_configured_handler;
        printf("Edge Sensor has transitioned to STATE_CONFIGURED.\n");

        // step 2: store edge sensor in NVS for recovery after deep sleep
        save_edge_sensor_to_nvs(edgeSensor);
        printf("Edge Sensor has been stored in NVS.\n");
    }
}

void state_configured_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_START_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_WORKING;
        edgeSensor->stateMachine->stateHandler = state_working_handler;
        printf("Edge Sensor has transitioned to STATE_WORKING.\n");

        // step 2: signal measurement task to start
        xSemaphoreGive(edgeSensor->stateMachine->stateSemaphore);
        printf("Measurement task has been signaled to start.\n");
    }
}

void state_working_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_STOP_COMMAND_RECEIVED)
    {
        edgeSensor->stateMachine->state = STATE_STOPPED;
        edgeSensor->stateMachine->stateHandler = state_stopped_handler;
        printf("Edge Sensor has transitioned to STATE_STOPPED.\n");
        xSemaphoreTake(edgeSensor->stateMachine->stateSemaphore, portMAX_DELAY);
        printf("Measurement task has been signaled to stop.\n");
    }
}

void state_stopped_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_START_COMMAND_RECEIVED)
    {
        edgeSensor->stateMachine->state = STATE_WORKING;
        edgeSensor->stateMachine->stateHandler = state_working_handler;
        printf("Edge Sensor has transitioned to STATE_WORKING.\n");
        xSemaphoreGive(edgeSensor->stateMachine->stateSemaphore);
        printf("Measurement task has been signaled to start.\n");
    }
    else if (event == EVENT_RESET_COMMAND_RECEIVED)
    {
        edgeSensor->stateMachine->state = STATE_INITIAL;
        edgeSensor->stateMachine->stateHandler = state_initial_handler;
        printf("Edge Sensor has transitioned to STATE_INITIAL.\n");
    }
}

/* Edge Sensor Functions */
void edge_sensor_init(EdgeSensor *edgeSensor, ES_Config *config, ES_StateMachine *stateMachine, char *deviceName)
{
    // step 0: initialize random number generator (test)
    init_random();

    // step 1: check if edge sensor has been configured
    uint8_t configured = 0;
    check_edge_sensor_configured(&configured);
    if (configured)
    {
        load_edge_sensor_from_nvs(edgeSensor, config, stateMachine, deviceName);
        return;
    }

    // step 2: initialize Edge Sensor
    edgeSensor->deviceName = deviceName;
    edgeSensor->commandHandler = es_command_handler;
    edgeSensor->config = config;
    edgeSensor->stateMachine = stateMachine;

    // step 3: initialize state machine
    edgeSensor->stateMachine->state = STATE_INITIAL;
    edgeSensor->stateMachine->stateHandler = state_initial_handler;
    edgeSensor->stateMachine->stateMutex = xSemaphoreCreateMutex();
    edgeSensor->stateMachine->stateSemaphore = xSemaphoreCreateBinary();
}

float edge_sensor_measure(EdgeSensor *edgeSensor)
{
    return get_random_float32();
}

float edge_sensor_predict(EdgeSensor *edgeSensor, float measurement)
{
    return get_random_float32();
}


void edge_sensor_sleep(EdgeSensor *edgeSensor)
{
    // Calculate sleep time in microseconds
    int64_t sleepTimeMicroseconds = (int64_t)edgeSensor->config->measurementIntervalMS * 1000;

    // Free predictive model allocated memory
    free(edgeSensor->config->predictiveModel);
            
    // Set the wake up time for the ESP32
    esp_sleep_enable_timer_wakeup(sleepTimeMicroseconds);

    // Go to deep sleep
    esp_deep_sleep_start();
}
