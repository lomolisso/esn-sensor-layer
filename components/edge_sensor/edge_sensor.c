#include "edge_sensor.h"

#include <stdio.h>  // for ESP_LOGI(TAG, )
#include <stdlib.h> // for rand()
#include <time.h>   // for time()

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "inttypes.h"
#include <string.h>
#include "mbedtls/base64.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "zlib.h"

static const char *TAG = "EDGE_SENSOR";

// Heap info imports
#include "esp_heap_caps.h"


void printHeapInfo()
{
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Free heap size: %zu bytes\n", freeHeap);
    size_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Largest free contiguous heap block: %zu bytes\n", largestFreeBlock);
}


/* Utils */
void from_uint64_to_string(uint64_t timestamp, char *buffer)
{
    /* Convert the timestamp to a string */
    sprintf(buffer, "%" PRIu64, timestamp);
}

uint64_t from_string_to_uint64(char *buffer)
{
    /* Convert the string to a uint64_t */
    uint64_t timestamp = strtoull(buffer, NULL, 0);
    return timestamp;
}

size_t get_b64_decoded_length(const char *encoded_str, size_t encoded_len)
{
    size_t padding = 0;
    if (encoded_len >= 2 && encoded_str[encoded_len - 1] == '=')
        padding++;
    if (encoded_len >= 2 && encoded_str[encoded_len - 2] == '=')
        padding++;

    // Calculate the decoded length
    return (encoded_len / 4) * 3 - padding;
}

/* Json Payloads */
void build_export_sensor_data_payload(cJSON *jsonPayload, SensorDataExport *sensorDataExport)
{
    // Add the "low_battery" field
    cJSON_AddBoolToObject(jsonPayload, SENSOR_DATA_LOW_BATTERY_FIELD, sensorDataExport->lowBattery);

    // Create the "reading" field
    cJSON *reading = cJSON_CreateString(sensorDataExport->b64_gzip_reading);
    cJSON_AddItemToObject(jsonPayload, SENSOR_DATA_READING_FIELD, reading);

    // Create the "inference_descriptor" field
    cJSON *inferenceDescriptor = cJSON_CreateObject();
    cJSON_AddNumberToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_INFERENCE_LAYER_FIELD, sensorDataExport->inferenceLayer);

    // Add the "prediction" field (handle null case)
    if (sensorDataExport->prediction)
    {
        cJSON_AddNumberToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_PREDICTION_FIELD, *(sensorDataExport->prediction));
    }
    else
    {
        cJSON_AddNullToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_PREDICTION_FIELD);
    }

    // Add the "send_timestamp" field (handle null case)
    if (sensorDataExport->sendTimestamp)
    {
        cJSON_AddNumberToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_SEND_TIMESTAMP_FIELD, *(sensorDataExport->sendTimestamp));
    }
    else
    {
        cJSON_AddNullToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_SEND_TIMESTAMP_FIELD);
    }

    // Add the "recv_timestamp" field (handle null case)
    if (sensorDataExport->recvTimestamp)
    {
        cJSON_AddNumberToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_RECV_TIMESTAMP_FIELD, *(sensorDataExport->recvTimestamp));
    }
    else
    {
        cJSON_AddNullToObject(inferenceDescriptor, SENSOR_DATA_INFERENCE_DESCRIPTOR_RECV_TIMESTAMP_FIELD);
    }

    // Add the "inference_descriptor" to the main payload
    cJSON_AddItemToObject(jsonPayload, SENSOR_DATA_INFERENCE_DESCRIPTOR_FIELD, inferenceDescriptor);
}

uint8_t *compress_bytes(uint8_t *data, size_t data_len, size_t *compressed_len)
{
    ESP_LOGI(TAG, "Starting Compression for %zu bytes\n", data_len);

    uLongf compressed_size = compressBound(data_len);
    uint8_t *compressed_data = (uint8_t *)malloc(compressed_size);
    
    printHeapInfo();
    
    
    
    int compress_status = compress(compressed_data, &compressed_size, (const Bytef *)data, data_len);
    if (compress_status == Z_OK)
    {
        ESP_LOGI(TAG, "Compression successful.\n");
        ESP_LOGI(TAG, "Original size: %lu\n", (unsigned long)data_len);
        ESP_LOGI(TAG, "Compressed size: %lu\n", (unsigned long)compressed_size);

        // Set the compressed length and return the compressed data
        *compressed_len = compressed_size;

        return compressed_data;
    }
    else
    {
        ESP_LOGI(TAG, "Compression failed!, status: %d\n", compress_status);
        return NULL;
    }
}


char *encode_base64(uint8_t *data, size_t data_len, size_t *b64_encoded_len)
{
    size_t b64_encoded_size = 0;
    int ret = 0;

    // First, get the required buffer size for the base64 encoded data
    ret = mbedtls_base64_encode(NULL, 0, &b64_encoded_size, data, data_len);
    if (ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL)
    {
        ESP_LOGE(TAG, "Failed to get the required buffer size for base64 encoding\n");
        return NULL;
    }

    // Allocate memory for the base64 encoded data
    char *b64_encoded_buffer = (char *)malloc(b64_encoded_size);
    if (b64_encoded_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for base64 encoded data\n");
        return NULL;
    }

    // Perform the actual base64 encoding
    ret = mbedtls_base64_encode((unsigned char *)b64_encoded_buffer, b64_encoded_size, &b64_encoded_size, data, data_len);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to encode data to base64\n");
        free(b64_encoded_buffer);
        return NULL;
    }

    // Set the output length and return the encoded buffer
    *b64_encoded_len = b64_encoded_size;
    return b64_encoded_buffer;
}



/* Reading Buffer */
float32_t **alloc_reading_buffer()
{
    // Allocate memory for the sequence length (array of pointers)
    float32_t **reading = (float32_t **)malloc(BMI270_SEQUENCE_LENGTH * sizeof(float32_t *));
    if (reading == NULL)
    {
        return NULL; // Memory allocation failed
    }

    // Allocate memory for each sample in the sequence
    for (int i = 0; i < BMI270_SEQUENCE_LENGTH; i++)
    {
        reading[i] = (float32_t *)malloc(BMI270_SAMPLE_SIZE * sizeof(float32_t));
        if (reading[i] == NULL)
        {
            // If allocation fails, free previously allocated memory and return NULL
            for (int j = 0; j < i; j++)
            {
                free(reading[j]);
            }
            free(reading);
            return NULL;
        }
    }

    return reading;
}

uint8_t *alloc_raw_reading_buffer()
{
    /*
    The BMI270 IMU sensor produces 6 elements per sample, each element is a int16_t, hence 
    we need 2 uint8_t to store each element. 

    ...| msb__acc_x | lsb__acc_x | msb__acc_y | lsb__acc_y | msb__acc_z | lsb__acc_z | msb__gyr_x | lsb__gyr_x | msb__gyr_y | lsb__gyr_y | msb__gyr_z | lsb__gyr_z |...    

    */
    uint8_t *reading = (uint8_t *)malloc(BMI270_SEQUENCE_LENGTH * BMI270_SAMPLE_SIZE * 2);
    if (reading == NULL)
    {
        return NULL; // Memory allocation failed
    }
    return reading;
}




void free_reading_buffer(float32_t **reading)
{
    // Frees the allocated buffer
    if (reading != NULL)
    {
        for (int i = 0; i < BMI270_SEQUENCE_LENGTH; i++)
        {
            free(reading[i]);
        }
        free(reading);
    }
}

void free_raw_reading_buffer(uint8_t *reading)
{
    // Frees the allocated buffer
    if (reading != NULL)
    {
        free(reading);
    }
}


/* NVS Edge Sensor Flags Functions */
esp_err_t _set_nvs_edge_sensor_flag(const char *key, uint8_t value)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_u8(nvs_handle, key, value);
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

esp_err_t set_nvs_edge_sensor_inf_flag(uint8_t value)
{
    return _set_nvs_edge_sensor_flag(NVS_INF_FLAG_KEY, value);
}

esp_err_t set_nvs_edge_sensor_model_flag(uint8_t value)
{
    return _set_nvs_edge_sensor_flag(NVS_MODEL_FLAG_KEY, value);
}

esp_err_t set_nvs_edge_sensor_config_flag(uint8_t value)
{
    return _set_nvs_edge_sensor_flag(NVS_CONFIG_FLAG_KEY, value);
}

esp_err_t set_nvs_edge_sensor_sleep_flag(uint8_t value)
{
    return _set_nvs_edge_sensor_flag(NVS_SLEEP_FLAG_KEY, value);
}

uint8_t _get_nvs_edge_sensor_flag(const char *key)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t flag = 0;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return 0;
    }

    err = nvs_get_u8(nvs_handle, key, &flag);
    if (err != ESP_OK)
    {
        return 0;
    }

    nvs_close(nvs_handle);

    return (flag == 1) ? 1 : 0;
}

uint8_t get_nvs_edge_sensor_inf_flag()
{
    return _get_nvs_edge_sensor_flag(NVS_INF_FLAG_KEY);
}

uint8_t get_nvs_edge_sensor_model_flag()
{
    return _get_nvs_edge_sensor_flag(NVS_MODEL_FLAG_KEY);
}

uint8_t get_nvs_edge_sensor_config_flag()
{
    return _get_nvs_edge_sensor_flag(NVS_CONFIG_FLAG_KEY);
}

uint8_t get_nvs_edge_sensor_sleep_flag()
{
    return _get_nvs_edge_sensor_flag(NVS_SLEEP_FLAG_KEY);
}

uint8_t get_nvs_bmi270_config_flag()
{
    return _get_nvs_edge_sensor_flag(NVS_BMI270_CONFIG_FLAG_KEY);
}

/* NVS Edge Sensor Struct Functions */
esp_err_t reset_nvs_edge_sensor()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    /*
    Setting the inf_flag to 0 indicates that the edge sensor has not been configured with an inference layer.
    This will cause the edge sensor's inference layer to be initialized as INFERENCE_LAYER_SENSOR.
    */
    err = nvs_set_u8(nvs_handle, NVS_INF_FLAG_KEY, 0);
    if (err != ESP_OK)
        return err;

    /*
    Setting the sleep_flag to 0 indicates that the edge sensor has not been sleeping.
    This will cause the edge sensor's state machine to be initialized in the initial state.
    */
    err = nvs_set_u8(nvs_handle, NVS_SLEEP_FLAG_KEY, 0);
    if (err != ESP_OK)
        return err;
    /*
    Setting the model_flag to 0 indicates that no predictive model has been provided.
    This will cause the edge sensor's predictive model to be initialized as a NULL pointer.
    */
    err = nvs_set_u8(nvs_handle, NVS_MODEL_FLAG_KEY, 0);
    if (err != ESP_OK)
        return err;

    /*
    Setting the config_flag to 0 indicates that the edge sensor has not been configured.
    This will cause the edge sensor's config to be initialized as a NULL pointer.
    */
    err = nvs_set_u8(nvs_handle, NVS_CONFIG_FLAG_KEY, 0);
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

    /* inference layer */
    ESP_LOGI(TAG, "Saving inference layer to NVS.\n");
    err = nvs_set_u8(nvs_handle, NVS_INFERENCE_LAYER_SENSOR_KEY, (uint8_t)edgeSensor->inferenceLayer);
    if (err != ESP_OK)
        return err;

    /* predictive model */
    if (edgeSensor->predictiveModel != NULL)
    {
        ESP_LOGI(TAG, "Saving predictive model to NVS.\n");
        err = nvs_set_u32(nvs_handle, NVS_SENSOR_MODEL_BYTESIZE_KEY, edgeSensor->predictiveModel->size);
        if (err != ESP_OK)
            return err;

        size_t model_size = edgeSensor->predictiveModel->size;
        ESP_LOGI(TAG, "Model size: %d bytes\n", model_size);
        err = nvs_set_blob(nvs_handle, NVS_SENSOR_MODEL_BYTES_KEY, (const void *)edgeSensor->predictiveModel->buffer, model_size);
        if (err != ESP_OK)
            return err;
    }

    /* config */
    if (edgeSensor->config != NULL)
    {
        ESP_LOGI(TAG, "Saving measurement interval to NVS.\n");
        err = nvs_set_u32(nvs_handle, NVS_SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_KEY, edgeSensor->config->measurementIntervalMS);
        if (err != ESP_OK)
            return err;
    }

    /* state machine */
    ESP_LOGI(TAG, "Saving state machine state to NVS.\n");
    err = nvs_set_u8(nvs_handle, NVS_SENSOR_STATE_KEY, (uint8_t)edgeSensor->stateMachine->state);
    if (err != ESP_OK)
        return err;

    /* remaining battery */
    ESP_LOGI(TAG, "Saving remaining battery to NVS.\n");
    err = nvs_set_u32(nvs_handle, NVS_SENSOR_REMAINING_BATTERY_KEY, edgeSensor->remainingBattery);
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

/* Edge Sensor Command Handler */
cJSON *_es_handle_set_sensor_state(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = NULL;
    char *strState = cJSON_GetObjectItem(payload, ES_PROPERTY_SENSOR_STATE)->valuestring;
    ES_State enumState = FROM_STRING_STATE_TO_ENUM(strState);
    ESP_LOGI(TAG, "Received SET sensor-state command with state: %s\n", strState);

    ES_Event event = enumState == STATE_LOCKED ? EVENT_CMD_SET_STATE_LOCKED_RECV : enumState == STATE_UNLOCKED ? EVENT_CMD_SET_STATE_UNLOCKED_RECV
                                                                                : enumState == STATE_WORKING    ? EVENT_CMD_SET_STATE_WORKING_RECV
                                                                                : enumState == STATE_IDLE       ? EVENT_CMD_SET_STATE_IDLE_RECV
                                                                                : enumState == STATE_ERROR      ? EVENT_CMD_SET_STATE_ERROR_RECV
                                                                                                                : EVENT_CMD_SET_STATE_INITIAL_RECV;
    edgeSensor->stateMachine->stateHandler(edgeSensor, event);

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_get_sensor_state(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = cJSON_CreateObject();
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    ESP_LOGI(TAG, "Received GET sensor-state command\n");

    ES_State enumState = stateMachine->state;
    char *strState = FROM_ENUM_STATE_TO_STRING(enumState);
    cJSON_AddStringToObject(response, ES_PROPERTY_SENSOR_STATE, strState);

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_set_sensor_config(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = NULL;
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    ES_State enumState = stateMachine->state;
    ESP_LOGI(TAG, "Received SET sensor-config command\n");

    if (edgeSensor->config == NULL)
    {
        edgeSensor->config = (ES_Config *)malloc(sizeof(ES_Config));
    }

    if (enumState == STATE_UNLOCKED)
    {
        cJSON *config = cJSON_GetObjectItem(payload, ES_PROPERTY_SENSOR_CONFIG);
        cJSON *measurementIntervalMS = cJSON_GetObjectItem(config, SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_FIELD);
        ESP_LOGI(TAG, "Payload: %s\n", cJSON_Print(payload));
        edgeSensor->config->measurementIntervalMS = cJSON_IsNumber(measurementIntervalMS) ? measurementIntervalMS->valueint : 1000;
        ESP_ERROR_CHECK(set_nvs_edge_sensor_config_flag(1));
    }

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_get_sensor_config(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = cJSON_CreateObject();
    ESP_LOGI(TAG, "Received GET sensor-config command\n");

    if (edgeSensor->config != NULL)
    {
        cJSON *configJson = cJSON_CreateObject();
        cJSON_AddNumberToObject(configJson, SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_FIELD, edgeSensor->config->measurementIntervalMS);
        cJSON_AddItemToObject(response, ES_PROPERTY_SENSOR_CONFIG, configJson);
    }

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_set_inference_layer(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = NULL;
    ES_StateMachine *stateMachine = edgeSensor->stateMachine;
    ES_State enumState = stateMachine->state;
    ESP_LOGI(TAG, "Received SET inference-layer command\n");

    if (enumState == STATE_UNLOCKED || enumState == STATE_WORKING)
    {
        cJSON *inferenceLayer = cJSON_GetObjectItem(payload, ES_PROPERTY_INFERENCE_LAYER);
        edgeSensor->inferenceLayer = (ES_InferenceLayer)cJSON_IsNumber(inferenceLayer) ? inferenceLayer->valueint : 0;
        ESP_ERROR_CHECK(set_nvs_edge_sensor_inf_flag(1));
    }

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_get_inference_layer(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = cJSON_CreateObject();
    ESP_LOGI(TAG, "Received GET inference-layer command\n");

    cJSON_AddNumberToObject(response, ES_PROPERTY_INFERENCE_LAYER, edgeSensor->inferenceLayer);

    cJSON_Delete(payload);
    return response;
}

cJSON *_es_handle_set_sensor_model(EdgeSensor *edgeSensor, cJSON *payload)
{
    cJSON *response = NULL;
    ESP_LOGI(TAG, "Received SET sensor-model command\n");

    /* Step 1: alloc space for ES_PredictiveModel Struct if needed */
    if (edgeSensor->predictiveModel == NULL)
    {
        edgeSensor->predictiveModel = (ES_PredictiveModel *)malloc(sizeof(ES_PredictiveModel));
    }

    /* Step 2: retrieve the predictive model data from the JSON payload */
    cJSON *predictiveModelJson = cJSON_GetObjectItem(payload, ES_PROPERTY_SENSOR_MODEL);

    // predictive model bytesize
    cJSON *JSON_predictiveModelSize = cJSON_GetObjectItem(predictiveModelJson, SENSOR_MODEL_BYTESIZE_FIELD);
    size_t model_size = cJSON_IsNumber(JSON_predictiveModelSize) ? JSON_predictiveModelSize->valueint : 0;

    // base64 string encoding the gzipped predictive model
    cJSON *JSON_predictiveModelB64 = cJSON_GetObjectItem(predictiveModelJson, SENSOR_MODEL_B64_FIELD);
    if (!cJSON_IsString(JSON_predictiveModelB64))
    {
        ESP_LOGI(TAG, "Predictive model is not a string.\n");
        free(edgeSensor->predictiveModel);
        edgeSensor->predictiveModel = NULL;
        return response;
    }
    char *b64_encoded_gzipped_model = JSON_predictiveModelB64->valuestring;

    /* Step 3: decode the base64 string into a byte array */
    size_t encoded_length = strlen(b64_encoded_gzipped_model);
    size_t decoded_length = get_b64_decoded_length(b64_encoded_gzipped_model, encoded_length);
    uint8_t *decoded_buffer = (uint8_t *)malloc(decoded_length);
    size_t written_bytes;

    int ret = mbedtls_base64_decode(
        decoded_buffer,
        decoded_length,
        &written_bytes,
        (const unsigned char *)b64_encoded_gzipped_model,
        encoded_length);
    if (ret != 0)
    {
        ESP_LOGI(TAG, "Failed to decode model, error code: %d\n", ret);
        free(decoded_buffer);
        free(edgeSensor->predictiveModel);
        edgeSensor->predictiveModel = NULL;
        return response;
    }


    /* Step 4: free JSON payload as it is no longer needed */
    cJSON_Delete(payload);

    /* Step 5: decompress the byte array using zlib */
    uLongf uncompressed_length = (uLongf)model_size;
    unsigned char *uncompressed_buffer = (unsigned char *)malloc(uncompressed_length);
    if (uncompressed_buffer == NULL)
    {
        ESP_LOGI(TAG, "Failed to allocate memory for uncompressed data\n");
        free(decoded_buffer);
        free(edgeSensor->predictiveModel);
        edgeSensor->predictiveModel = NULL;
        return response;
    }

    int status = uncompress(uncompressed_buffer, &uncompressed_length, decoded_buffer, (uLong)written_bytes);
    if (status != Z_OK)
    {
        ESP_LOGI(TAG, "Failed to decompress model, error code: %d\n", status);
        free(decoded_buffer);
        free(uncompressed_buffer);
        free(edgeSensor->predictiveModel);
        edgeSensor->predictiveModel = NULL;
        return response;
    }


    /* Step 6: free the decoded buffer as it is no longer needed */
    free(decoded_buffer);

    /* Step 7: log some information */
    ESP_LOGI(TAG, "Base64 encoded gzipped model size: %d bytes\n", encoded_length);
    ESP_LOGI(TAG, "Decoded buffer size: %d bytes\n", decoded_length);
    ESP_LOGI(TAG, "Bytes written into decoded buffer: %d bytes\n", written_bytes);
    ESP_LOGI(TAG, "Uncompressed buffer size: %lu bytes\n", uncompressed_length);

    /* Step 8: update the ES_PredictiveModel struct */
    edgeSensor->predictiveModel->buffer = uncompressed_buffer;
    edgeSensor->predictiveModel->size = uncompressed_length;
    edgeSensor->predictiveModel->predictionHistory = 0;
    edgeSensor->predictiveModel->predictionCounter = 0;

    /* Step 9: set the model flag in NVS */
    ESP_ERROR_CHECK(set_nvs_edge_sensor_model_flag(1));

    return response;
}

cJSON *es_command_handler(EdgeSensor *edgeSensor, const char *propertyName, const char *cmdMethod, cJSON *payload)
{
    if (strcmp(propertyName, ES_PROPERTY_SENSOR_STATE) == 0)
    {
        if (strcmp(cmdMethod, ES_METHOD_SET) == 0)
        {
            return _es_handle_set_sensor_state(edgeSensor, payload);
        }
        else if (strcmp(cmdMethod, ES_METHOD_GET) == 0)
        {
            return _es_handle_get_sensor_state(edgeSensor, payload);
        }
        else
        {
            ESP_LOGI(TAG, "Command method not recognized.\n");
            return NULL;
        }
    }
    else if (strcmp(propertyName, ES_PROPERTY_SENSOR_CONFIG) == 0)
    {
        if (strcmp(cmdMethod, ES_METHOD_SET) == 0)
        {
            return _es_handle_set_sensor_config(edgeSensor, payload);
        }
        else if (strcmp(cmdMethod, ES_METHOD_GET) == 0)
        {
            return _es_handle_get_sensor_config(edgeSensor, payload);
        }
        else
        {
            ESP_LOGI(TAG, "Command method not recognized.\n");
            return NULL;
        }
    }
    else if (strcmp(propertyName, ES_PROPERTY_INFERENCE_LAYER) == 0)
    {
        if (strcmp(cmdMethod, ES_METHOD_SET) == 0)
        {
            return _es_handle_set_inference_layer(edgeSensor, payload);
        }
        else if (strcmp(cmdMethod, ES_METHOD_GET) == 0)
        {
            return _es_handle_get_inference_layer(edgeSensor, payload);
        }
        else
        {
            ESP_LOGI(TAG, "Command method not recognized.\n");
            return NULL;
        }
    }
    else if (strcmp(propertyName, ES_PROPERTY_SENSOR_MODEL) == 0)
    {
        if (strcmp(cmdMethod, ES_METHOD_SET) == 0)
        {
            return _es_handle_set_sensor_model(edgeSensor, payload);
        }
        else
        {
            ESP_LOGI(TAG, "Command method not recognized.\n");
            return NULL;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Command not recognized.\n");
        return NULL;
    }
}

/* State Machine Transition Functions */
void _transition_to_initial(EdgeSensor *edgeSensor)
{
    edgeSensor->stateMachine->state = STATE_INITIAL;
    edgeSensor->stateMachine->stateHandler = state_initial_handler;
    ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_INITIAL.\n");

    reset_nvs_edge_sensor();
}

void _transition_to_unlocked(EdgeSensor *edgeSensor)
{
    edgeSensor->stateMachine->state = STATE_UNLOCKED;
    edgeSensor->stateMachine->stateHandler = state_unlocked_handler;
    ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_UNLOCKED.\n");
}

void _transition_to_locked(EdgeSensor *edgeSensor)
{
    edgeSensor->stateMachine->state = STATE_LOCKED;
    edgeSensor->stateMachine->stateHandler = state_locked_handler;
    ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_LOCKED.\n");
}

void _transition_to_working(EdgeSensor *edgeSensor)
{
    ES_InferenceLayer inferenceLayer = edgeSensor->inferenceLayer;
    uint8_t config_flag = get_nvs_edge_sensor_config_flag();
    uint8_t model_flag = get_nvs_edge_sensor_model_flag();

    bool is_configured = config_flag == 1;
    bool has_model = model_flag == 1;
    bool on_device_inference = inferenceLayer == INFERENCE_LAYER_SENSOR;

    if (is_configured && (!on_device_inference || (on_device_inference && has_model)))
    {
        edgeSensor->stateMachine->state = STATE_WORKING;
        edgeSensor->stateMachine->stateHandler = state_working_handler;
        ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_WORKING.\n");
    }
    else
    {
        ESP_LOGI(TAG, "Edge Sensor has not been configured or does not have a model.\n");
    }
}

void _transition_to_idle(EdgeSensor *edgeSensor)
{
    edgeSensor->stateMachine->state = STATE_IDLE;
    edgeSensor->stateMachine->stateHandler = state_idle_handler;
    ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_IDLE.\n");
}

void _transition_to_error(EdgeSensor *edgeSensor)
{
    edgeSensor->stateMachine->state = STATE_ERROR;
    edgeSensor->stateMachine->stateHandler = state_error_handler;
    ESP_LOGI(TAG, "Edge Sensor has transitioned to STATE_ERROR.\n");
}

/* State Machine Event handlers */
void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    ESP_LOGI(TAG, "Edge Sensor is in STATE_INITIAL and received event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
    switch (event)
    {
    case EVENT_ES_BOOTED_UP:
        _transition_to_unlocked(edgeSensor);
        break;
    case EVENT_ES_LOAD_STATE_LOCKED_FROM_NVS:
        _transition_to_locked(edgeSensor);
        break;
    case EVENT_ES_LOAD_STATE_WORKING_FROM_NVS:
        _transition_to_working(edgeSensor);
        break;
    case EVENT_ES_LOAD_STATE_IDLE_FROM_NVS:
        _transition_to_idle(edgeSensor);
        break;
    case EVENT_ES_LOAD_STATE_ERROR_FROM_NVS:
        _transition_to_error(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_INITIAL and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

void state_unlocked_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    ESP_LOGI(TAG, "Edge Sensor is in STATE_UNLOCKED and received event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
    switch (event)
    {
    case EVENT_CMD_SET_STATE_LOCKED_RECV:
        _transition_to_locked(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_WORKING_RECV:
        _transition_to_working(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_INITIAL_RECV:
        _transition_to_initial(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_UNLOCKED and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

void state_locked_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    switch (event)
    {
    case EVENT_CMD_SET_STATE_UNLOCKED_RECV:
        _transition_to_unlocked(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_WORKING_RECV:
        _transition_to_working(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_INITIAL_RECV:
        _transition_to_initial(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_LOCKED and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

void state_working_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    switch (event)
    {
    case EVENT_CMD_SET_STATE_IDLE_RECV:
        _transition_to_idle(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_ERROR_RECV:
        _transition_to_error(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_INITIAL_RECV:
        _transition_to_initial(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_WORKING and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

void state_idle_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    switch (event)
    {
    case EVENT_CMD_SET_STATE_WORKING_RECV:
        _transition_to_working(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_ERROR_RECV:
        _transition_to_error(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_INITIAL_RECV:
        _transition_to_initial(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_IDLE and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

void state_error_handler(EdgeSensor *edgeSensor, ES_Event event)
{
    switch (event)
    {
    case EVENT_CMD_SET_STATE_UNLOCKED_RECV:
        _transition_to_unlocked(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_WORKING_RECV:
        _transition_to_working(edgeSensor);
        break;
    case EVENT_CMD_SET_STATE_INITIAL_RECV:
        _transition_to_initial(edgeSensor);
        break;
    default:
        ESP_LOGI(TAG, "Edge Sensor is in STATE_ERROR and cannot handle event %s.\n", FROM_ENUM_EVENT_TO_STRING(event));
        break;
    }
}

/* Edge Sensor Functions */
esp_err_t edge_sensor_init(EdgeSensor *edgeSensor, char *deviceName, uint8_t sleep_flag)
{
    // step 1: initialize edge sensor
    edgeSensor->deviceName = deviceName;
    edgeSensor->inferenceLayer = INFERENCE_LAYER_SENSOR;
    edgeSensor->commandHandler = es_command_handler;
    
    // step 2: initialize member structs with default values
    edgeSensor->stateMachine = (ES_StateMachine *)malloc(sizeof(ES_StateMachine));
    edgeSensor->stateMachine->state = STATE_INITIAL;
    edgeSensor->stateMachine->stateHandler = state_initial_handler;

    edgeSensor->predictiveModel = NULL;
    edgeSensor->config = NULL;

    edgeSensor->bmi270 = (ES_BMI270 *)malloc(sizeof(ES_BMI270));
    edgeSensor->bmi270->readingBuffer = NULL;
    edgeSensor->bmi270->rawReadingBuffer = NULL;
    edgeSensor->bmi270->configSize = 0;
    edgeSensor->bmi270->configBuffer = NULL;

    // step 3: update member structs from NVS if necessary
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open_from_partition(EDGE_SENSOR_PARTITION, EDGE_SENSOR_PARTITION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;


    // =================
    // REMAINING BATTERY
    // =================


    if (!sleep_flag)
    {
        ESP_LOGI(TAG, "Setting remaining battery to BATTERY_CAPACITY.\n");
        edgeSensor->remainingBattery = BATTERY_CAPACITY;
    }
    else {
        ESP_LOGI(TAG, "Reading remaining battery from NVS.\n");
        uint32_t remaining_battery_buffer;
        err = nvs_get_u32(nvs_handle, NVS_SENSOR_REMAINING_BATTERY_KEY, &remaining_battery_buffer);
        if (err != ESP_OK)
            return err;
        edgeSensor->remainingBattery = remaining_battery_buffer;
    }


    // =================
    // INFERENCE LAYER
    // =================


    /* Read inference layer from NVS */
    uint8_t inf_flag = get_nvs_edge_sensor_inf_flag();
    if (inf_flag)
    {
        ESP_LOGI(TAG, "Reading inference layer from NVS.\n");
        uint8_t inf_layer_buffer;
        err = nvs_get_u8(nvs_handle, NVS_INFERENCE_LAYER_SENSOR_KEY, &inf_layer_buffer);
        if (err != ESP_OK)
            return err;
        edgeSensor->inferenceLayer = (ES_InferenceLayer)inf_layer_buffer;
    }
    edgeSensor->inferenceLayer = INFERENCE_LAYER_GATEWAY; // TODO: REMOVE THIS!, I re-defined inference layer for DEBUG, disables loading model.

    // ===================
    // PREDICTIVE MODEL
    // ===================


    /* Read ES_PredictiveModel from NVS */
    uint8_t model_flag = get_nvs_edge_sensor_model_flag();
    if (model_flag && edgeSensor->inferenceLayer == INFERENCE_LAYER_SENSOR) // only load model if inference layer is on device
    {
        ESP_LOGI(TAG, "Allocating memory for ES_PredictiveModel struct.\n");
        edgeSensor->predictiveModel = (ES_PredictiveModel *)malloc(sizeof(ES_PredictiveModel));

        // Read the size of the predictive model
        ESP_LOGI(TAG, "Reading predictive model size from NVS.\n");
        uint32_t model_size_buffer;
        err = nvs_get_u32(nvs_handle, NVS_SENSOR_MODEL_BYTESIZE_KEY, &model_size_buffer);
        if (err != ESP_OK)
            return err;

        size_t model_size = (size_t)model_size_buffer;
        edgeSensor->predictiveModel->size = model_size;

        // Read the predictive model byte array
        ESP_LOGI(TAG, "Allocating memory for ES_PredictiveModel->buffer.\n");
        void *model_buffer = malloc(model_size); // model MUST be 8-byte aligned
        if (model_buffer == NULL)
        {
            ESP_LOGI(TAG, "Failed to allocate memory for ES_PredictiveModel->buffer.\n");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Reading predictive model from NVS.\n");
        err = nvs_get_blob(nvs_handle, NVS_SENSOR_MODEL_BYTES_KEY, model_buffer, &model_size);
        if (err != ESP_OK)
        {
            free(model_buffer); // Free the allocated memory in case of error
            return err;
        }
        edgeSensor->predictiveModel->buffer = (uint8_t *)model_buffer;
    }


    // =================
    // BMI270 IMU
    // =================


    /* Initialize BMI270 reading buffer */
    ESP_LOGI(TAG, "Current inference layer: %d\n", edgeSensor->inferenceLayer);
    if (edgeSensor->inferenceLayer == INFERENCE_LAYER_SENSOR)
    {
        ESP_LOGI(TAG, "Allocating memory for ES_BMI270->readingBuffer.\n");
        edgeSensor->bmi270->readingBuffer = alloc_reading_buffer();
    }
    else {
        ESP_LOGI(TAG, "Allocating memory for ES_BMI270->rawReadingBuffer.\n");
        edgeSensor->bmi270->rawReadingBuffer = alloc_raw_reading_buffer();
    }
    /* Read ES_BMI270 Config from NVS if necessary */
    uint8_t bmi270_config_flag = get_nvs_bmi270_config_flag();
    if (bmi270_config_flag && !sleep_flag) // only load config if first cycle, i.e. havent slept yet
    {
        ESP_LOGI(TAG, "Reading BMI270 config size from NVS.\n");
        uint32_t bmi270_config_size;
        err = nvs_get_u32(nvs_handle, NVS_BMI270_CONFIG_SIZE_KEY, &bmi270_config_size);
        if (err != ESP_OK)
            return err;
        edgeSensor->bmi270->configSize = bmi270_config_size;
        
        ESP_LOGI(TAG, "Allocating memory for ES_BMI270->configBuffer.\n");
        uint8_t *bmi270ConfigBuffer = (uint8_t *)malloc(bmi270_config_size);

        ESP_LOGI(TAG, "Reading BMI270 config from NVS.\n");
        size_t buffer_size = (size_t)bmi270_config_size;
        err = nvs_get_blob(nvs_handle, NVS_BMI270_CONFIG_KEY, bmi270ConfigBuffer, &buffer_size);
        if (err != ESP_OK)
            return err;
        edgeSensor->bmi270->configBuffer = bmi270ConfigBuffer;
    }

    // ===================
    // EDGE SENSOR CONFIG
    // ===================


    /* Read ES_Config from NVS */
    uint8_t config_flag = get_nvs_edge_sensor_config_flag();
    if (config_flag)
    {
        ESP_LOGI(TAG, "Reading config from NVS.\n");
        edgeSensor->config = (ES_Config *)malloc(sizeof(ES_Config));

        err = nvs_get_u32(nvs_handle, NVS_SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_KEY, &edgeSensor->config->measurementIntervalMS);
        if (err != ESP_OK)
            return err;
    }

    /* Read ES_State from NVS */
    sleep_flag = 1; // TODO: REMOVE THIS!, I re-defined sleep flag for DEBUG, enables loading state.
    if (sleep_flag)
    {
        ESP_LOGI(TAG, "Reading state from NVS.\n");
        uint8_t state_buffer;
        err = nvs_get_u8(nvs_handle, NVS_SENSOR_STATE_KEY, &state_buffer);
        if (err != ESP_OK)
            return err;
        ES_State state = (ES_State)state_buffer;
        switch (state)
        {
        case STATE_LOCKED:
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_ES_LOAD_STATE_LOCKED_FROM_NVS);
            break;
        case STATE_WORKING:
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_ES_LOAD_STATE_WORKING_FROM_NVS);
            break;
        case STATE_IDLE:
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_ES_LOAD_STATE_IDLE_FROM_NVS);
            break;
        case STATE_ERROR:
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_ES_LOAD_STATE_ERROR_FROM_NVS);
            break;
        default:
            edgeSensor->stateMachine->stateHandler(edgeSensor, EVENT_ES_BOOTED_UP); // default to unlocked
            break;
        }
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

    /* free state machine allocated memory */
    free(edgeSensor->stateMachine);

    if (edgeSensor->bmi270->configBuffer != NULL)
        free(edgeSensor->bmi270->configBuffer);

    free(edgeSensor->bmi270);
}

void get_hardcoded_measurement(float32_t **readingBuffer)
{
    float32_t hardcodedSample[BMI270_SAMPLE_SIZE] = {
        -4.417302,  0.620098,  10.017340, -0.201868,  2.382863,  0.002663
    };
    
    for (int i = 0; i < BMI270_SEQUENCE_LENGTH; i++)
    {
        for (int j = 0; j < BMI270_SAMPLE_SIZE; j++)
        {
            readingBuffer[i][j] = hardcodedSample[j];
        }
    }
}


void edge_sensor_measure(EdgeSensor *edgeSensor)
{
    get_hardcoded_measurement(edgeSensor->bmi270->readingBuffer);
}

void edge_sensor_update_inference_layer(EdgeSensor *edgeSensor)
{
    /*
        Sensor Adaptive Inference Heuristic

        M_t: prediction history at time step t
        sigma(M_t): number of abnormal predictions in history at time step t
        psi_s: threshold for abnormal predictions, if greater than psi_s, set inference layer to gateway

        u_t = self._pred_state_counter
        assert u_t >= len(self._prediction_history)
        m = PREDICTION_HISTORY_LENGTH
        sigma_M_t = sum(self._prediction_history)
        low_battery = self.is_device_low_battery()
        psi_s = ABNORMAL_PREDICTION_THRESHOLD

        if low_battery: # b_t < psi_b
            return INFERENCE_LAYER_GATEWAY
        else: # b_t >= psi_b
            if u_t < m: # history not full => sensor
                return INFERENCE_LAYER_SENSOR
            else: # u_t >= m
                if sigma_M_t >= psi_s: # abnormal predictions => gateway
                    return INFERENCE_LAYER_GATEWAY
                else: # normal predictions => sensor
                    return INFERENCE_LAYER_SENSOR
    */
    uint32_t ut = edgeSensor->predictiveModel->predictionCounter;
    uint32_t Mt = edgeSensor->predictiveModel->predictionHistory;
    uint32_t m = PREDICTION_HISTORY_LENGTH;
    uint8_t sigma_Mt = SIGMA(Mt);
    uint8_t low_battery = IS_LOW_BATTERY(edgeSensor->remainingBattery);
    uint8_t psi_s = ABNORMAL_PREDICTION_THRESHOLD;

    if (ADAPTIVE_INFERENCE)
        edgeSensor->inferenceLayer = low_battery ? INFERENCE_LAYER_GATEWAY : \
                                    ut < m ? INFERENCE_LAYER_SENSOR : \
                                    sigma_Mt >= psi_s ? INFERENCE_LAYER_GATEWAY : \
                                    INFERENCE_LAYER_SENSOR;
}

void edge_sensor_sleep(EdgeSensor *edgeSensor)
{
    /* update remaining battery */
    ES_InferenceLayer inferenceLayer = edgeSensor->inferenceLayer;
    uint32_t cycleConsumption = inferenceLayer == INFERENCE_LAYER_SENSOR ? INFERENCE_CYCLE_CONSUMPTION : NORMAL_CYCLE_CONSUMPTION;
    edgeSensor->remainingBattery = edgeSensor->remainingBattery - cycleConsumption;

    /* set sleeping flag in NVS */
    ESP_ERROR_CHECK(set_nvs_edge_sensor_sleep_flag(1));

    /* calculate sleep time in microseconds */
    int32_t measurementIntervalMS = (edgeSensor->config == NULL) ? 10000 : edgeSensor->config->measurementIntervalMS;
    int64_t sleepTimeMicroseconds = (int64_t)measurementIntervalMS * 1000;

    /* set the wake up time for the ESP32 */
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleepTimeMicroseconds));

    /* free allocated memory */
    free_edge_sensor(edgeSensor);

    /* go to deep sleep */
    ESP_LOGI(TAG, "Deep sleeping for %lld seconds\n", sleepTimeMicroseconds / 1000000);
    esp_deep_sleep_start();
}

