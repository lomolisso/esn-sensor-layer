#include "edge_sensor.h"
#include "mqtt_client.h"


/* MQTT Client*/
esp_mqtt_client_handle_t mqtt_init();
void start_mqtt(esp_mqtt_client_handle_t client, EdgeSensor* edgeSensor);

/* MQTT Topics */
void get_publish_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic);
void get_subscribe_topic(char *topic_buffer, size_t buffer_size, const char *device_name, const char *topic, const char* method);

/* MQTT Publish */
void publish_measurement(esp_mqtt_client_handle_t client, char *device_name, float measurement, float *prediction_ptr);
void publish_mqtt_conn_status(esp_mqtt_client_handle_t client, char *device_name, bool status);