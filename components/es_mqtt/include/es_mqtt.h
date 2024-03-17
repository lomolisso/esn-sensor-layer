#include "edge_sensor.h"
#include "mqtt_client.h"


/* MQTT Client*/
esp_mqtt_client_handle_t mqtt_init();
void start_mqtt(esp_mqtt_client_handle_t client, EdgeSensor* edgeSensor);

/* MQTT Topics */
void get_publish_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic);
void get_subscribe_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic, const char* method);

/* MQTT Publish */
void publish_measurement(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload);
void publish_prediction_log(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload);
void publish_mqtt_conn_status(esp_mqtt_client_handle_t client, char *device_name, bool status);
void publish_gateway_prediction_request(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload);
void publish_cloud_prediction_request(esp_mqtt_client_handle_t client, char *device_name, cJSON* jsonPayload);
void publish_prediction_request(esp_mqtt_client_handle_t client, char *deviceName, cJSON *jsonPayload, uint8_t predLayers);
