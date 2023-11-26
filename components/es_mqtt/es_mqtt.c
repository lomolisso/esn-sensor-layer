#include "es_mqtt.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_event.h"

#include "esp_log.h"
#include <cJSON.h>

#define TOPIC_BUFFER_MAX_SIZE 100

#define COMMAND_NAME_MAX_SIZE 32
#define COMMAND_METHOD_MAX_SIZE 4
#define COMMAND_UUID_MAX_SIZE 37

static const char *CONFIG_BROKER_URL = "mqtt://192.168.4.135:1883";
static const char *TAG = "ES_MQTT";

/* MQTT Utils */

void get_command_name(const char *topic, char *buffer, size_t buffer_size) {
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);

    // find first '/' and replace it with '\0'
    char *first_slash = strchr(buffer, '/');
    if (first_slash != NULL)
        *first_slash = '\0';
}

void get_command_method(const char *topic, char *buffer, size_t buffer_size) {
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}

void get_command_uuid(const char *topic, char *buffer, size_t buffer_size) {
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%*[^/]/%%%zu[0-9a-fA-F-]", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}



/* MQTT Topics */

void get_publish_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic) {
    snprintf(topic_buffer, buffer_size, "incoming/data/%s/%s", device_name, topic);
}

void get_subscribe_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic, const char* method) {
    snprintf(topic_buffer, buffer_size, "command/%s/%s/%s/#", device_name, topic, method);
}

void get_command_response_topic(char *topic_buffer, size_t buffer_size, char *uuid) {
    snprintf(topic_buffer, buffer_size, "command/response/%s", uuid);
}


/* MQTT Publish */

void publish_command_response(esp_mqtt_client_handle_t client, char *device_name, char *command_uuid, char *response) {
    /* Get publish topic */
    char command_response_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_command_response_topic(command_response_topic_buffer, TOPIC_BUFFER_MAX_SIZE, command_uuid);

    /* Publish command response to MQTT broker */
    int msg_id = esp_mqtt_client_publish(client, command_response_topic_buffer, response, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s, msg_id=%d", response, command_response_topic_buffer, msg_id);
}


void publish_measurement(esp_mqtt_client_handle_t client, char *device_name, float measurement, float *prediction_ptr) {
    /* Create JSON payload using cJSON */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "measurement", measurement);

    /* If prediction is not null, add it to the payload */
    if (prediction_ptr != NULL)
        cJSON_AddNumberToObject(root, "prediction", *prediction_ptr);
    else
        cJSON_AddNullToObject(root, "prediction");

    char *payload = cJSON_Print(root);
    cJSON_Delete(root); // Free the cJSON object

    /* Get publish topic */
    char measurement_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(measurement_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, "edge-sensor-data");
    
    /* Publish measurement to MQTT broker */
    int msg_id = esp_mqtt_client_publish(client, measurement_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published %s on topic %s, msg_id=%d", payload, measurement_topic_buffer, msg_id);
}

void publish_mqtt_conn_status(esp_mqtt_client_handle_t client, char *device_name, bool status) {
    /* Create JSON payload using cJSON */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "edge-sensor-mqtt-conn-status", status);
    char *payload = cJSON_Print(root);
    cJSON_Delete(root); // Free the cJSON object

    /* Get publish topic */
    char mqtt_connection_status_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_publish_topic(mqtt_connection_status_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, "edge-sensor-mqtt-conn-status");

    /* Publish connection status to MQTT broker */
    int msg_id = esp_mqtt_client_publish(client, mqtt_connection_status_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "published on topic %s, msg_id=%d", mqtt_connection_status_topic_buffer, msg_id);
}

/* MQTT Subscribe */

void subscribe_to_config_topics(esp_mqtt_client_handle_t client, char *device_name) {
    /* List of config topics */
    char *config_topics[] = {
        "config-measurement-interval-ms",
        "config-run-prediction",
        "config-predictive-model",
        "config-done"
    };

    char config_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    int num_topics = sizeof(config_topics) / sizeof(config_topics[0]);
    for (int i = 0; i < num_topics; i++) {
        /* Get subscribe topic */
        get_subscribe_topic(config_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, config_topics[i], "set");

        /* Subscribe to config topic */
        int msg_id = esp_mqtt_client_subscribe(client, config_command_topic_buffer, 0);

        ESP_LOGI(TAG, "successfully subscribed to topic '%s', msg_id=%d", config_command_topic_buffer, msg_id);
    }
}

void subscribe_to_start_topic(esp_mqtt_client_handle_t client, char *device_name) {
    /* Get subscribe topic */
    char start_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_subscribe_topic(start_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, "edge-sensor-start", "set");

    /* Subscribe to start topic */
    int msg_id = esp_mqtt_client_subscribe(client, start_command_topic_buffer, 0);

    ESP_LOGI(TAG, "successfully subscribed to topic '%s', msg_id=%d", start_command_topic_buffer, msg_id);
}

void subscribe_to_stop_topic(esp_mqtt_client_handle_t client, char *device_name) {
    /* Get subscribe topic */
    char stop_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_subscribe_topic(stop_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, "edge-sensor-stop", "set");

    /* Subscribe to stop topic */
    int msg_id = esp_mqtt_client_subscribe(client, stop_command_topic_buffer, 0);

    ESP_LOGI(TAG, "successfully subscribed to topic '%s', msg_id=%d", stop_command_topic_buffer, msg_id);
}

void subscribe_to_reset_topic(esp_mqtt_client_handle_t client, char *device_name) {
    /* Get subscribe topic */
    char reset_command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_subscribe_topic(reset_command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, "edge-sensor-reset", "set");

    /* Subscribe to reset topic */
    int msg_id = esp_mqtt_client_subscribe(client, reset_command_topic_buffer, 0);

    ESP_LOGI(TAG, "successfully subscribed to topic '%s', msg_id=%d", reset_command_topic_buffer, msg_id);
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    /* Retrieve Edge Sensor pointer from handler_args */
    EdgeSensor *edgeSensor = (EdgeSensor *) handler_args;

    /* Retrieve event and client */
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        // Publish connection status to MQTT broker
        publish_mqtt_conn_status(client, edgeSensor->deviceName, true);

        /* Subscribe to commands topics */
        subscribe_to_config_topics(client, edgeSensor->deviceName);
        subscribe_to_start_topic(client, edgeSensor->deviceName);
        subscribe_to_stop_topic(client, edgeSensor->deviceName);
        subscribe_to_reset_topic(client, edgeSensor->deviceName);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        /* Get command name, method and uuid */
        char command_name_buffer[COMMAND_NAME_MAX_SIZE];
        get_command_name(event->topic, command_name_buffer, COMMAND_NAME_MAX_SIZE);
        
        char command_method_buffer[COMMAND_METHOD_MAX_SIZE];
        get_command_method(event->topic, command_method_buffer, COMMAND_METHOD_MAX_SIZE);
        
        char command_uuid_buffer[COMMAND_UUID_MAX_SIZE];
        get_command_uuid(event->topic, command_uuid_buffer, COMMAND_UUID_MAX_SIZE);

        /* Call edge sensor command handler */
        edgeSensor->commandHandler(edgeSensor, command_name_buffer, command_method_buffer, event->data);

        /* Publish command response to MQTT broker */
        char *response = event->data; // TODO: replace by actual response.
        publish_command_response(client, edgeSensor->deviceName, command_uuid_buffer, response);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_mqtt_client_handle_t mqtt_init() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    return esp_mqtt_client_init(&mqtt_cfg);
}

void start_mqtt(esp_mqtt_client_handle_t client, EdgeSensor *edgeSensor) {
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void *) edgeSensor);
    esp_mqtt_client_start(client);
}