#include "es_mqtt.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include <cJSON.h>

static const char *CONFIG_BROKER_URL = "mqtt://192.168.4.135:1883";
static const char *TAG = "ES_MQTT";

/* MQTT Utils */
bool _validate_json(cJSON *jsonPayload) 
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

void _get_property_name(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);

    // find first '/' and replace it with '\0'
    char *first_slash = strchr(buffer, '/');
    if (first_slash != NULL)
        *first_slash = '\0';
}

void _get_cmd_method(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%%zus", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}

void _get_cmd_uuid(const char *topic, char *buffer, size_t buffer_size)
{
    char format[50];
    snprintf(format, sizeof(format), "command/%%*[^/]/%%*[^/]/%%*[^/]/%%%zu[0-9a-fA-F-]", buffer_size - 1);
    sscanf(topic, format, buffer);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
}


/* MQTT Topics */

void get_export_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *export_name)
{
    snprintf(topic_buffer, buffer_size, "export/%s/%s", device_name, export_name);
}

void get_response_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *property_name, const char *cmd_method, const char *cmd_uuid)
{
    snprintf(topic_buffer, buffer_size, "response/%s/%s/%s/%s", device_name, property_name, cmd_method, cmd_uuid);
}

void get_command_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *property_name, const char *cmd_method, const char *cmd_uuid)
{
    snprintf(topic_buffer, buffer_size, "command/%s/%s/%s/%s", device_name, property_name, cmd_method, cmd_uuid);
}

/* MQTT Publish */
void publish_export_sensor_data(esp_mqtt_client_handle_t client, char *device_name, cJSON *jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char export_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_export_topic(export_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, ES_EXPORT_SENSOR_DATA);

    /* Publish measurement to MQTT broker */
    esp_mqtt_client_publish(client, export_topic_buffer, payload, 0, 0, 0);

    ESP_LOGI(TAG, "Published export sensor-data on topic %s", export_topic_buffer);
    free(payload);
}

void publish_cmd_response(esp_mqtt_client_handle_t client, char *device_name, char *property_name, char *cmd_uuid, cJSON *jsonPayload)
{
    char *payload = cJSON_Print(jsonPayload);

    /* Get publish topic */
    char response_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_response_topic(response_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, property_name, CMD_METHOD_GET, cmd_uuid);

    /* Publish command response to MQTT broker */
    esp_mqtt_client_publish(client, response_topic_buffer, payload, 0, 1, 0);

    ESP_LOGI(TAG, "Published command response on topic %s", response_topic_buffer);
    free(payload);
}

/* MQTT Subscribe */
void _subscribe_to_command(esp_mqtt_client_handle_t client, char *device_name, char *property_name, char *method)
{
    /* Get subscribe topic */
    char command_topic_buffer[TOPIC_BUFFER_MAX_SIZE];
    get_command_topic(command_topic_buffer, TOPIC_BUFFER_MAX_SIZE, device_name, property_name, method, MQTT_TOPIC_WILDCARD);

    /* Subscribe to command topic */
    esp_mqtt_client_subscribe(client, command_topic_buffer, 1);

    ESP_LOGI(TAG, "Subscribed to command topic %s", command_topic_buffer);
}

void subscribe_to_property_commands(esp_mqtt_client_handle_t client, char *device_name)
{
    /* Subscribe to property 'sensor-state' commands */
    _subscribe_to_command(client, device_name, ES_PROPERTY_SENSOR_STATE, CMD_METHOD_SET);
    _subscribe_to_command(client, device_name, ES_PROPERTY_SENSOR_STATE, CMD_METHOD_GET);

    /* Subscribe to property 'inference-layer' commands */
    _subscribe_to_command(client, device_name, ES_PROPERTY_INFERENCE_LAYER, CMD_METHOD_SET);
    _subscribe_to_command(client, device_name, ES_PROPERTY_INFERENCE_LAYER, CMD_METHOD_GET);

    /* Subscribe to property 'sensor-config' commands */
    _subscribe_to_command(client, device_name, ES_PROPERTY_SENSOR_CONFIG, CMD_METHOD_SET);
    _subscribe_to_command(client, device_name, ES_PROPERTY_SENSOR_CONFIG, CMD_METHOD_GET);

    /* Subscribe to property 'sensor-model' command */
    _subscribe_to_command(client, device_name, ES_PROPERTY_SENSOR_MODEL, CMD_METHOD_SET);
}

/* MQTT Event Handler */
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    MqttHandlerArgs *mqttHandlerArgs = (MqttHandlerArgs *)handler_args;
    
    /* Retrieve Edge Sensor pointer from mqtt handler args */
    EdgeSensor *edgeSensor = mqttHandlerArgs->edgeSensor;
    TaskHandle_t measurementTaskHandle = mqttHandlerArgs->measurementTaskHandle;

    /* Retrieve event and client */
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "ESP32 connected to MQTT broker");

        /* Subscribe to property commands */
        subscribe_to_property_commands(client, edgeSensor->deviceName);

        /* Notify measurement task */
        if (measurementTaskHandle != NULL) {
            xTaskNotifyGive(measurementTaskHandle);
        } else {
            ESP_LOGE(TAG, "Measurement task handle is NULL");
        }
        break;

    case MQTT_EVENT_DATA:
        /* Get CommandSemaphore */
        xSemaphoreTake(edgeSensor->mutexSemaphore, portMAX_DELAY);

        /* Get topic and payload */
        char *topic = event->topic;
        char *payload = event->data;
        ESP_LOGI(TAG, "ESP32 received MQTT message from topic %s", topic);

        /* Get property name, cmd method and cmd uuid */
        char propertyName[PROPERTY_NAME_MAX_SIZE];
        _get_property_name(topic, propertyName, PROPERTY_NAME_MAX_SIZE);

        char cmdMethod[CMD_METHOD_MAX_SIZE];
        _get_cmd_method(topic, cmdMethod, CMD_METHOD_MAX_SIZE);

        char cmdUUID[CMD_UUID_MAX_SIZE];
        _get_cmd_uuid(topic, cmdUUID, CMD_UUID_MAX_SIZE);

        /* Parse payload into JSON */
        cJSON *jsonPayload = cJSON_Parse(payload);
        if (!_validate_json(jsonPayload))
        {
            ESP_LOGI(TAG, "Invalid JSON payload: %s\n", payload);
            return;
        }

        /* Verify cmdMethod is either SET or GET */
        if (strcmp(cmdMethod, CMD_METHOD_SET) != 0 && strcmp(cmdMethod, CMD_METHOD_GET) != 0)
        {
            ESP_LOGI(TAG, "Invalid command method: %s\n", cmdMethod);
            return;
        }

        cJSON *response = edgeSensor->commandHandler(edgeSensor, propertyName, cmdMethod, jsonPayload);
        if (response != NULL && strcmp(cmdMethod, CMD_METHOD_GET))
        {
            publish_cmd_response(client, edgeSensor->deviceName, propertyName, cmdUUID, response);
            cJSON_Delete(response);
        }

        /* Return CommandSemaphore */
        xSemaphoreGive(edgeSensor->mutexSemaphore);
        break;
    default:
        break;
    }
}

esp_mqtt_client_handle_t mqtt_init()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .buffer.size = 16 * 1024,
        .buffer.out_size = 1024,
        .session.disable_clean_session = 1,
    };

    return esp_mqtt_client_init(&mqtt_cfg);
}

void start_mqtt(esp_mqtt_client_handle_t client, MqttHandlerArgs *mqttHandlerArgs)
{
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void *)mqttHandlerArgs);
    esp_mqtt_client_start(client);
}