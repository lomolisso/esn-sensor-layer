#include "edge_sensor.h"

#include <stdio.h>  // for printf()
#include <stdlib.h> // for rand()
#include <time.h>   // for time()

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include <string.h>
#include "mbedtls/base64.h"

#include "esp_sleep.h"

/* Utils */
bool starts_with(const char *pre, const char *str)
{
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

/* Random number generator */
void init_random()
{
    srand((unsigned)time(NULL));
}

float get_random_float()
{   
    // returns a random float between 0 and 1
    return (float)rand() / (float)RAND_MAX;
}

/* NVS */
esp_err_t set_nvs_edge_sensor_model_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_u8(nvs_handle, "model_flag", 1);
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

esp_err_t set_nvs_edge_sensor_config_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_u8(nvs_handle, "config_flag", 1);
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

esp_err_t set_nvs_edge_sensor_sleep_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_u8(nvs_handle, "sleep_flag", 1);
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

uint8_t get_nvs_edge_sensor_model_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t has_model = 0;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return 0;
    }

    err = nvs_get_u8(nvs_handle, "model_flag", &has_model);
    if (err != ESP_OK)
    {
        return 0;
    }

    nvs_close(nvs_handle);

    return (has_model == 1) ? 1 : 0;
}

uint8_t get_nvs_edge_sensor_config_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t configured = 0;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return 0;
    }

    err = nvs_get_u8(nvs_handle, "config_flag", &configured);
    if (err != ESP_OK)
    {
        return 0;
    }

    nvs_close(nvs_handle);

    return (configured == 1) ? 1 : 0;
}

uint8_t get_nvs_edge_sensor_sleep_flag()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t sleeping = 0;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return 0;
    }

    err = nvs_get_u8(nvs_handle, "sleep_flag", &sleeping);
    if (err != ESP_OK)
    {
        return 0;
    }

    nvs_close(nvs_handle);

    return (sleeping == 1) ? 1 : 0;
}

esp_err_t reset_nvs_edge_sensor()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    /*
    Setting the sleep_flag to 0 indicates that the edge sensor has not been sleeping.
    This will cause the edge sensor's state machine to be initialized in the initial state.
    */
    err = nvs_set_u8(nvs_handle, "sleep_flag", 0);
    if (err != ESP_OK)
        return err;
    /*
    Setting the model_flag to 0 indicates that no predictive model has been provided.
    This will cause the edge sensor's predictive model to be initialized as a NULL pointer.
    */
    err = nvs_set_u8(nvs_handle, "model_flag", 0);
    if (err != ESP_OK)
        return err;

    /*
    Setting the config_flag to 0 indicates that the edge sensor has not been configured.
    This will cause the edge sensor's config to be initialized as a NULL pointer.
    */
    err = nvs_set_u8(nvs_handle, "config_flag", 0);
    if (err != ESP_OK)
        return err;

    nvs_close(nvs_handle);

    return ESP_OK;
}

esp_err_t save_edge_sensor_to_nvs(EdgeSensor *edgeSensor)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    /* predictive model */
    if (edgeSensor->predictiveModel != NULL)
    {
        err = nvs_set_u32(nvs_handle, "predModelSize", edgeSensor->predictiveModel->size);
        if (err != ESP_OK)
            return err;

        size_t model_size = edgeSensor->predictiveModel->size;
        err = nvs_set_blob(nvs_handle, "predModel", (const void *)edgeSensor->predictiveModel->buffer, model_size);
        if (err != ESP_OK)
            return err;
    }

    /* config */
    if (edgeSensor->config != NULL)
    {
        err = nvs_set_u32(nvs_handle, "workInterval", edgeSensor->config->measurementIntervalMS);
        if (err != ESP_OK)
            return err;
    }

    /* state machine */
    err = nvs_set_u8(nvs_handle, "state", (uint8_t)edgeSensor->stateMachine->state);
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

/* Edge Sensor Predictive Model Command Handler */
void es_predictive_model_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonPayload)
{
    if (edgeSensor->predictiveModel == NULL)
    {
        set_nvs_edge_sensor_model_flag();
        edgeSensor->predictiveModel = (ES_PredictiveModel *)malloc(sizeof(ES_PredictiveModel));
    }

    if (strcmp(commandName, PM_SIZE_COMMAND) == 0)
    {
        cJSON *JSON_predictiveModelSize = cJSON_GetObjectItem(jsonPayload, commandName);
        edgeSensor->predictiveModel->size = cJSON_IsNumber(JSON_predictiveModelSize) ? JSON_predictiveModelSize->valueint : 0;
    }
    else if (strcmp(commandName, PM_B64_COMMAND) == 0)
    {
        cJSON *JSON_predictiveModelB64 = cJSON_GetObjectItem(jsonPayload, commandName);
        if (!cJSON_IsString(JSON_predictiveModelB64))
        {
            printf("Predictive model is not a string.\n");
            return;
        }

        // buffer to store the decoded bytes
        size_t model_size = edgeSensor->predictiveModel->size;
        uint8_t *model_buffer = (uint8_t *)malloc(model_size); // model MUST be 8-byte aligned
        if (model_buffer == NULL)
        {
            printf("Failed to allocate memory for model buffer.\n");
            return;
        }

        char *b64_encoded_model = JSON_predictiveModelB64->valuestring;
        size_t b64_encoded_model_size = strlen(b64_encoded_model);
        size_t written_bytes;

        // Decode the base64 string
        int ret = mbedtls_base64_decode(
            model_buffer,
            model_size,
            &written_bytes,
            (const unsigned char *)b64_encoded_model,
            b64_encoded_model_size);

        if (ret != 0)
        {
            printf("Failed to decode model, error code: %d\n", ret);
            free(model_buffer);
            return;
        }

        printf("Model b64 encoded size: %d bytes\n", b64_encoded_model_size);
        printf("Model b64 encoded: %s\n", b64_encoded_model);
        printf("Expected Model size: %d bytes\n", model_size);
        printf("Actual Model size: %d bytes\n", written_bytes);

        edgeSensor->predictiveModel->buffer = model_buffer;
    }
    else
    {
        printf("Predictive Model Command '%s' not recognized.\n", commandName);
    }
}

/* Edge Sensor Config Command Handler */
void es_config_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonPayload)
{
    if (edgeSensor->config == NULL)
    {
        set_nvs_edge_sensor_config_flag();
        edgeSensor->config = (ES_Config *)malloc(sizeof(ES_Config));
    }

    if (strcmp(commandName, CONFIG_MEASUREMENT_INTERVAL_MS_COMMAND) == 0)
    {
        cJSON *JSON_measurementIntervalMS = cJSON_GetObjectItem(jsonPayload, commandName);
        edgeSensor->config->measurementIntervalMS = cJSON_IsNumber(JSON_measurementIntervalMS) ? JSON_measurementIntervalMS->valueint : 1000;
    }
    else
    {
        printf("Config command '%s' not recognized.\n", commandName);
    }
}

/* Edge Sensor State Machine Command Handler */
void es_state_machine_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonPayload)
{
    if (strcmp(commandName, SM_READY_COMMAND) == 0)
    {
        /* handle EVENT_READY_COMMAND_RECEIVED */
        edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_READY_COMMAND_RECEIVED);
    }
    else if (strcmp(commandName, SM_START_COMMAND) == 0)
    {
        /* handle EVENT_START_COMMAND_RECEIVED */
        edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_START_COMMAND_RECEIVED);
    }
    else if (strcmp(commandName, SM_STOP_COMMAND) == 0)
    {
        /* handle EVENT_STOP_COMMAND_RECEIVED */
        edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_STOP_COMMAND_RECEIVED);
    }
    else if (strcmp(commandName, SM_RESET_COMMAND) == 0)
    {
        /* handle EVENT_RESET_COMMAND_RECEIVED */
        edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_RESET_COMMAND_RECEIVED);
    }
    else
    {
        printf("State Machine Command '%s' not recognized.\n", commandName);
    }
}

/* Edge Sensor Command Handler */
void es_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonPayload)
{
    if (starts_with("config", commandName))
    {
        es_config_command_handler(edgeSensor, commandName, jsonPayload);
    }
    else if (starts_with("predictive-model", commandName))
    {
        es_predictive_model_command_handler(edgeSensor, commandName, jsonPayload);
    }
    else if (starts_with("state-machine", commandName))
    {
        es_state_machine_command_handler(edgeSensor, commandName, jsonPayload);
    }
    else
    {
        printf("Command '%s' not recognized.\n", commandName);
    }

    /* update pending commands */
    int *pendingCommands = edgeSensor->pendingCommands;
    *pendingCommands = *pendingCommands - 1;
    if (*pendingCommands == 0)
    {
        xSemaphoreGive(edgeSensor->commandSemaphore);
    }
}

/* State Machine Event handlers */
void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_READY_COMMAND_RECEIVED)
    {
        edgeSensor->stateMachine->state = STATE_READY;
        edgeSensor->stateMachine->stateHandler = state_ready_handler;
        printf("Edge Sensor has transitioned to STATE_READY.\n");
    }
    else if (event == EVENT_RESET_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_INITIAL;
        edgeSensor->stateMachine->stateHandler = state_initial_handler;
        printf("Edge Sensor has transitioned to STATE_INITIAL.\n");

        // step 2: reset edge sensor configuration from NVS
        reset_nvs_edge_sensor();
    }
    else
    {
        printf("Edge Sensor is in STATE_INITIAL and cannot handle event %d.\n", event);
    }
}

void state_ready_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_START_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_WORKING;
        edgeSensor->stateMachine->stateHandler = state_working_handler;
        printf("Edge Sensor has transitioned to STATE_WORKING.\n");
    }
    else if (event == EVENT_RESET_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_INITIAL;
        edgeSensor->stateMachine->stateHandler = state_initial_handler;
        printf("Edge Sensor has transitioned to STATE_INITIAL.\n");

        // step 2: reset edge sensor configuration from NVS
        reset_nvs_edge_sensor();
    }
    else
    {
        printf("Edge Sensor is in STATE_READY and cannot handle event %d.\n", event);
    }
}

void state_working_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_STOP_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_STOPPED;
        edgeSensor->stateMachine->stateHandler = state_stopped_handler;
        printf("Edge Sensor has transitioned to STATE_STOPPED.\n");
    }
    else if (event == EVENT_RESET_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_INITIAL;
        edgeSensor->stateMachine->stateHandler = state_initial_handler;
        printf("Edge Sensor has transitioned to STATE_INITIAL.\n");

        // step 2: reset edge sensor configuration from NVS
        reset_nvs_edge_sensor();
    }
    else
    {
        printf("Edge Sensor is in STATE_WORKING and cannot handle event %d.\n", event);
    }
}

void state_stopped_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    if (event == EVENT_START_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_WORKING;
        edgeSensor->stateMachine->stateHandler = state_working_handler;
        printf("Edge Sensor has transitioned to STATE_WORKING.\n");
    }
    else if (event == EVENT_RESET_COMMAND_RECEIVED)
    {
        // step 1: update state machine
        edgeSensor->stateMachine->state = STATE_INITIAL;
        edgeSensor->stateMachine->stateHandler = state_initial_handler;
        printf("Edge Sensor has transitioned to STATE_INITIAL.\n");

        // step 2: reset edge sensor configuration from NVS
        reset_nvs_edge_sensor();
    }
    else
    {
        printf("Edge Sensor is in STATE_STOPPED and cannot handle event %d.\n", event);
    }
}

/* Edge Sensor Functions */
esp_err_t edge_sensor_init(EdgeSensor *edgeSensor, char *deviceName, uint8_t sleep_flag, uint8_t model_flag, uint8_t config_flag)
{
    // step 1: initialize edge sensor
    edgeSensor->deviceName = deviceName;
    edgeSensor->commandHandler = es_command_handler;
    edgeSensor->pendingCommands = (int *)malloc(sizeof(int));
    *(edgeSensor->pendingCommands) = 0;
    edgeSensor->commandSemaphore = xSemaphoreCreateBinary();

    // step 2: initialize member structs with default values
    edgeSensor->predictiveModel = NULL;
    edgeSensor->config = NULL;
    edgeSensor->stateMachine = (ES_StateMachine *)malloc(sizeof(ES_StateMachine));
    edgeSensor->stateMachine->state = STATE_INITIAL;
    edgeSensor->stateMachine->stateHandler = state_initial_handler;

    // step 3: update member structs from NVS if necessary
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;

    if (config_flag)
    {
        printf("Edge Sensor is configured. Reading config from NVS.\n");
        edgeSensor->config = (ES_Config *)malloc(sizeof(ES_Config));

        err = nvs_get_u32(nvs_handle, "workInterval", &edgeSensor->config->measurementIntervalMS);
        if (err != ESP_OK)
            return err;
    }

    if (model_flag)
    {
        printf("Edge Sensor has a predictive model. Reading model from NVS.\n");
        edgeSensor->predictiveModel = (ES_PredictiveModel *)malloc(sizeof(ES_PredictiveModel));

        printf("Reading model size from NVS.\n");
        uint32_t model_size;
        err = nvs_get_u32(nvs_handle, "predModelSize", &model_size);
        if (err != ESP_OK)
            return err;

        edgeSensor->predictiveModel->size = (size_t)model_size;

        size_t required_size;
        err = nvs_get_blob(nvs_handle, "predModel", NULL, &required_size);
        if (err != ESP_OK)
            return err;

        // Check if required_size is equal to model_size
        if (required_size != edgeSensor->predictiveModel->size)
        {
            printf("Model size in NVS does not match expected size.\n");
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
        printf("Model size: %d bytes\n", required_size);

        printf("Reading model from NVS.\n");
        void *model_buffer = malloc(required_size); // model MUST be 8-byte aligned
        if (model_buffer == NULL) {
            return ESP_ERR_NO_MEM;
        }
        err = nvs_get_blob(nvs_handle, "predModel", model_buffer, &required_size);
        if (err != ESP_OK) {
            free(model_buffer);  // Free the allocated memory in case of error
            return err;
        }
        edgeSensor->predictiveModel->buffer = (uint8_t *)model_buffer;
    }

    if (sleep_flag)
    {
        printf("Edge Sensor was sleeping. Reading state from NVS.\n");
        uint8_t state_buffer;
        err = nvs_get_u8(nvs_handle, "state", &state_buffer);
        if (err != ESP_OK)
            return err;

        ES_State state = (ES_State)state_buffer;
        edgeSensor->stateMachine->state = state;

        if (state == STATE_INITIAL)
        {
            edgeSensor->stateMachine->stateHandler = state_initial_handler;
        }
        else if (state == STATE_READY)
        {
            edgeSensor->stateMachine->stateHandler = state_ready_handler;
        }
        else if (state == STATE_WORKING)
        {
            edgeSensor->stateMachine->stateHandler = state_working_handler;
        }
        else if (state == STATE_STOPPED)
        {
            edgeSensor->stateMachine->stateHandler = state_stopped_handler;
        }
        printf("Edge Sensor has transitioned to STATE_%d.\n", state);
    }

    // Close NVS
    nvs_close(nvs_handle);

    return ESP_OK;
}

void free_edge_sensor(EdgeSensor *edgeSensor)
{
    /* free predictive model allocated memory */
    if (edgeSensor->predictiveModel != NULL)
    {
        free(edgeSensor->predictiveModel->buffer);
        free(edgeSensor->predictiveModel);
    }

    /* free config allocated memory */
    if (edgeSensor->config != NULL)
    {
        free(edgeSensor->config);
    }

    /* free edge sensor allocated memory */
    free(edgeSensor->stateMachine);
    free(edgeSensor->pendingCommands);
    vSemaphoreDelete(edgeSensor->commandSemaphore);
}

float edge_sensor_measure(EdgeSensor *edgeSensor)
{
    return get_random_float();
}

void edge_sensor_sleep(EdgeSensor *edgeSensor)
{
    /* set sleeping flag in NVS */
    ESP_ERROR_CHECK(set_nvs_edge_sensor_sleep_flag());

    /* calculate sleep time in microseconds */
    int32_t measurementIntervalMS = (edgeSensor->config == NULL) ? 10000 : edgeSensor->config->measurementIntervalMS;
    int64_t sleepTimeMicroseconds = (int64_t)measurementIntervalMS * 1000;

    /* set the wake up time for the ESP32 */
    esp_sleep_enable_timer_wakeup(sleepTimeMicroseconds);

    /* free allocated memory */
    free_edge_sensor(edgeSensor);

    /* go to deep sleep */
    printf("Deep sleeping for %lld seconds\n", sleepTimeMicroseconds / 1000000);
    esp_deep_sleep_start();
}
