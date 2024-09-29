#include "edge_sensor.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct mqttHandlerArgs
{
    EdgeSensor *edgeSensor;
    TaskHandle_t measurementTaskHandle;
} MqttHandlerArgs;

/* MQTT Topics */
#define MQTT_TOPIC_WILDCARD "#"

/* Sizes */
#define TOPIC_BUFFER_MAX_SIZE 100
#define PROPERTY_NAME_MAX_SIZE 32
#define CMD_METHOD_MAX_SIZE 4
#define CMD_UUID_MAX_SIZE 37

/* Allowed command methods */
#define CMD_METHOD_SET "set"
#define CMD_METHOD_GET "get"

/* MQTT Client*/
esp_mqtt_client_handle_t mqtt_init();
void start_mqtt(esp_mqtt_client_handle_t client, MqttHandlerArgs *mqttHandlerArgs);

/* Publish topics */
void get_export_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *export_name);
void get_response_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *property_name, const char *cmd_method, const char *cmd_uuid);

/* Subscribe topics */
void get_command_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *property_name, const char *cmd_method, const char *cmd_uuid);

/* MQTT Publish */
void publish_export_sensor_data(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload);
void publish_cmd_response(esp_mqtt_client_handle_t client, char *device_name, char *property_name, char *cmd_uuid, cJSON* jsonPayload);
