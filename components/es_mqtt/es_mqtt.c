#include "es_mqtt.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_event.h"

#include "esp_log.h"
#include <cJSON.h>

/* Sizes */
#define TOPIC_BUFFER_MAX_SIZE 100
#define COMMAND_NAME_MAX_SIZE 32
#define COMMAND_METHOD_MAX_SIZE 4
#define COMMAND_UUID_MAX_SIZE 37

/* Allowed command methods */
#define COMMAND_METHOD_SET "set"

/* Enqueued commands */
#define REQUEST_PENDING_COMMANDS "request-pending-commands"
#define RESPONSE_PENDING_COMMANDS "response-pending-commands"

/* Publish topics */
#define EXPORT_DATA "export-data"
#define PREDICTION_LOG "debug-prediction-log"

static const char *CONFIG_BROKER_URL = "mqtt://192.168.4.1:1883";
static const char *TAG = "ES_MQTT";

/* MQTT Utils */

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

void get_command_name(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);

    // find first '/' and replace it with '\0'
    char *first_slash = strchr(buffer, '/');
    if (first_slash != NULL)
        *first_slash = '\0';
}

void get_command_method(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}

void get_command_uuid(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%*[^/]/%%%zu[0-9a-fA-F-]", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}

/* MQTT Topics */

void get_publish_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic)
{
    snprintf(topic_buffer, buffer_size, "incoming/data/%s/%s", device_name, topic);
}

void get_subscribe_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic, const char *method)
{
    snprintf(topic_buffer, buffer_size, "command/%s/%s/%s/#", device_name, topic, method);
}

void get_command_response_topic(char *topic_buffer, size_t buffer_size, char *uuid)
{
    snprintf(topic_buffer, buffer_size, "command/response/%s", uuid);
}

/* MQTT Publish */

void publish_command_response(esp_mqtt_client_handle_t client, char *device_name, char *command_uuid, char *response)
{
    /* Get publish topic */
    char command_response_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_command_response_topic(command_response_topic_buffer, TOPIC_BUFFER_MAX_SIZE, command_uuid);

    /* Publish command response to MQTT broker */
    esp_mqtt_client_publish(client, command_response_topic_buffer, response, 0, 1, 0);

    ESP_LOGI(TAG, "published command response on topic %s", command_response_topic_buffer);
}

void publish_request_pending_commands(esp_mqtt_client_handle_t client, char *device_name, int pending_commands)
{
    /* Create JSON payload using cJSON */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, REQUEST_PENDING_COMMANDS, pending_commands);

    char *payload = cJSON_Print(root);
    cJSON_Delete(root); // Free the cJSON object

    /* Get publish topic */
    char pending_commands_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(pending_commands_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, REQUEST_PENDING_COMMANDS);

    /* Publish pending commands to MQTT broker */
    esp_mqtt_client_publish(client, pending_commands_topic_buffer, payload, 0, 2, 0);

    ESP_LOGI(TAG, "published %s on topic %s", payload, pending_commands_topic_buffer);
    free(payload);
}

void publish_measurement(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char measurement_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(measurement_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, EXPORT_DATA);

    /* Publish measurement to MQTT broker */
    esp_mqtt_client_publish(client, measurement_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s", payload, measurement_topic_buffer);
    free(payload);
}

void publish_cloud_prediction_request(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char prediction_request_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(prediction_request_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, CLOUD_PREDICTION_REQUEST);

    /* Publish measurement to MQTT broker */
    esp_mqtt_client_publish(client, prediction_request_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s", payload, prediction_request_topic_buffer);
    free(payload);
}

void publish_gateway_prediction_request(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char prediction_request_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(prediction_request_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, GATEWAY_PREDICTION_REQUEST);

    /* Publish measurement to MQTT broker */
    esp_mqtt_client_publish(client, prediction_request_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s", payload, prediction_request_topic_buffer);
    free(payload);
}

void publish_prediction_request(esp_mqtt_client_handle_t client, char *deviceName, cJSON *jsonPayload, uint8_t predLayers) {
    if (PREDICTION_ON_GATEWAY(predLayers)) {
        publish_gateway_prediction_request(client, deviceName, jsonPayload);
    }
    if (PREDICTION_ON_CLOUD(predLayers)) {
        publish_cloud_prediction_request(client, deviceName, jsonPayload);
    }
}

void publish_prediction_log(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char prediction_log_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(prediction_log_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, PREDICTION_LOG);

    /* Publish measurement to MQTT broker */
    esp_mqtt_client_publish(client, prediction_log_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s", payload, prediction_log_topic_buffer);
    free(payload);
}

/* MQTT Subscribe */

void subscribe_to_pending_commands_topic(esp_mqtt_client_handle_t client, char *device_name)
{
    /* Get subscribe topic */
    char pending_commands_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_subscribe_topic(pending_commands_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, RESPONSE_PENDING_COMMANDS, COMMAND_METHOD_SET);

    /* Subscribe to pending commands topic */
    esp_mqtt_client_subscribe(client, pending_commands_topic_buffer, 0);

    ESP_LOGI(TAG, "successfully subscribed to pending commands topic");
}

void subscribe_to_state_machine_topics(esp_mqtt_client_handle_t client, char *device_name)
{
    /* List of state machine topics */
    char *state_machine_topics[] = {
        SM_READY_COMMAND,
        SM_START_COMMAND,
        SM_STOP_COMMAND,
        SM_RESET_COMMAND
    };

    char state_machine_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    int num_topics = sizeof(state_machine_topics) / sizeof(state_machine_topics[0]);
    for (int i = 0; i < num_topics; i++)
    {
        /* Get subscribe topic */
        get_subscribe_topic(state_machine_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, state_machine_topics[i], COMMAND_METHOD_SET);

        /* Subscribe to state machine topic */
        esp_mqtt_client_subscribe(client, state_machine_command_topic_buffer, 0);

    }
    ESP_LOGI(TAG, "successfully subscribed to state machine topics");
}

void subscribe_to_predictive_model_topics(esp_mqtt_client_handle_t client, char *device_name)
{
    /* List of predictive model topics */
    char *predictive_model_topics[] = {
        PREDICTIVE_MODEL_COMMAND,
        PREDICTION_COMMAND
    };

    char predictive_model_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    int num_topics = sizeof(predictive_model_topics) / sizeof(predictive_model_topics[0]);
    for (int i = 0; i < num_topics; i++)
    {
        /* Get subscribe topic */
        get_subscribe_topic(predictive_model_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, predictive_model_topics[i], COMMAND_METHOD_SET);

        /* Subscribe to predictive model topic */
        esp_mqtt_client_subscribe(client, predictive_model_command_topic_buffer, 0);

    }
    ESP_LOGI(TAG, "successfully subscribed to predictive model topics");
}

void subscribe_to_config_topics(esp_mqtt_client_handle_t client, char *device_name)
{
    /* List of config topics */
    char *config_topics[] = {
        CONFIG_DEVICE_COMMAND
    };

    char config_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    int num_topics = sizeof(config_topics) / sizeof(config_topics[0]);
    for (int i = 0; i < num_topics; i++)
    {
        /* Get subscribe topic */
        get_subscribe_topic(config_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, config_topics[i], COMMAND_METHOD_SET);

        /* Subscribe to config topic */
        esp_mqtt_client_subscribe(client, config_command_topic_buffer, 0);

    }
    ESP_LOGI(TAG, "successfully subscribed to config topics");
}


void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    /* Retrieve Edge Sensor pointer from handler_args */
    EdgeSensor *edgeSensor = (EdgeSensor *)handler_args;

    /* Retrieve event and client */
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        /* Subscribe to state machine topics */
        subscribe_to_state_machine_topics(client, edgeSensor->deviceName);

        /* Subscribe to predictive model topics */
        subscribe_to_predictive_model_topics(client, edgeSensor->deviceName);

        /* Subscribe to config topics */
        subscribe_to_config_topics(client, edgeSensor->deviceName);

        /* Subscribe to pending commands topic */
        subscribe_to_pending_commands_topic(client, edgeSensor->deviceName);

        /* Publish pending commands request to MQTT broker */
        publish_request_pending_commands(client, edgeSensor->deviceName, 0);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // print topic and length
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);

        // print data and length
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);

        char *topic = event->topic;
        char *payload = event->data;

        /* Get command name, method and uuid */
        char commandName[COMMAND_NAME_MAX_SIZE];
        get_command_name(topic, commandName, COMMAND_NAME_MAX_SIZE);

        char commandMethod[COMMAND_METHOD_MAX_SIZE];
        get_command_method(topic, commandMethod, COMMAND_METHOD_MAX_SIZE);

        char commandUUID[COMMAND_UUID_MAX_SIZE];
        get_command_uuid(topic, commandUUID, COMMAND_UUID_MAX_SIZE);

        /* only support set commands */
        if (strcmp(commandMethod, COMMAND_METHOD_SET) != 0)
        {
            printf("Invalid command method: %s\n", commandMethod);
            return;
        }

        /* parse payload into JSON */
        cJSON *jsonPayload = cJSON_Parse(payload);
        if (!validate_json(jsonPayload))
        {
            printf("Invalid JSON payload: %s\n", payload);
            return;
        }

        if (strcmp(commandName, RESPONSE_PENDING_COMMANDS) == 0)
        {
            /* handle pending commands */
            cJSON *JSON_pendingCommands = cJSON_GetObjectItem(jsonPayload, commandName);
            int pendingCommands = cJSON_IsNumber(JSON_pendingCommands) ? JSON_pendingCommands->valueint : 0;
            *(edgeSensor->pendingCommands) = pendingCommands;
            printf("Edge Sensor has %d pending commands.\n", pendingCommands);
            if (pendingCommands == 0)
            {
                xSemaphoreGive(edgeSensor->commandSemaphore);
            }
        }
        else 
        {
            /* call edge sensor command handler */
            printf("Calling edge sensor command handler for command %s\n", commandName);
            edgeSensor->commandHandler(edgeSensor, commandName, jsonPayload);
        }
        
        /* free JSON payload */
        cJSON_Delete(jsonPayload);

        if (ESN_DEBUG_MODE && edgeSensor->predictionLogJson != NULL) {
            /* Publish prediction log to MQTT broker */
            publish_prediction_log(client, edgeSensor->deviceName, edgeSensor->predictionLogJson);
            cJSON_Delete(edgeSensor->predictionLogJson);
            
            /* Reset predictionLogJson to NULL */
            edgeSensor->predictionLogJson = NULL;
        }
        
        /* Publish command response to MQTT broker */
        cJSON *commandResponse = cJSON_CreateObject();
        cJSON_AddStringToObject(commandResponse, commandName, "success");
        char *response = cJSON_Print(commandResponse);
        publish_command_response(client, edgeSensor->deviceName, commandUUID, response);
        free(response);
        cJSON_Delete(commandResponse);
        break;
    default:
        break;
    }
}

esp_mqtt_client_handle_t mqtt_init()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .buffer.size = 12 * 1024,
        .buffer.out_size = 1024,
    };

    return esp_mqtt_client_init(&mqtt_cfg);
}

void start_mqtt(esp_mqtt_client_handle_t client, EdgeSensor *edgeSensor)
{
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void *)edgeSensor);
    esp_mqtt_client_start(client);
}