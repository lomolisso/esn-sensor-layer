#ifndef EDGE_SENSOR_H
#define EDGE_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "cJSON.h"

/* Debug mode*/
#define ESN_DEBUG_MODE 1

/* NVS */
#define EDGE_SENSOR_PARTITION "edge_sensor_data"
#define EDGE_SENSOR_PARTITION_NAMESPACE "edge_sensor_ns"

/* Predictive model commands */
#define PREDICTIVE_MODEL_COMMAND "device-predictive-model"
#define PM_SIZE_COMMAND "predictive-model-size"
#define PM_B64_COMMAND "predictive-model-b64"

/* Config commands */
#define CONFIG_DEVICE_COMMAND "device-config"
#define CONFIG_MEASUREMENT_INTERVAL_COMMAND "config-measurement-interval-ms"

/* State machine commands */
#define SM_READY_COMMAND "state-machine-ready"
#define SM_START_COMMAND "state-machine-start"
#define SM_STOP_COMMAND "state-machine-stop"
#define SM_RESET_COMMAND "state-machine-reset"

/* Prediction commands */
#define CLOUD_PREDICTION_REQUEST "cloud-prediction-request"
#define GATEWAY_PREDICTION_REQUEST "gateway-prediction-request"

#define PREDICTION_COMMAND "debug-prediction-command"
#define PRED_SOURCE_LAYER_COMMAND "pred-cmd-source-layer"
#define PRED_TIMESTAMP_COMMAND "pred-cmd-request-timestamp"
#define PRED_MEASUREMENT_COMMAND "pred-cmd-measurement"
#define PRED_PREDICTION_COMMAND "pred-cmd-prediction"

/* Forward Declarations */
typedef struct edgeSensor EdgeSensor;
typedef struct _ES_StateMachine ES_StateMachine;
typedef struct _ES_PredictiveModel ES_PredictiveModel;
typedef struct _ES_Config ES_Config;

/* Edge Sensor States */
typedef enum _ES_State {
    STATE_INITIAL,
    STATE_READY,
    STATE_WORKING,
    STATE_STOPPED,
} ES_State;

/* Edge Sensor Events */
typedef enum _ES_Event {
    EVENT_READY_COMMAND_RECEIVED,
    EVENT_START_COMMAND_RECEIVED,
    EVENT_STOP_COMMAND_RECEIVED,
    EVENT_RESET_COMMAND_RECEIVED,
} ES_Event;

/* Function Pointer Types using Forward Declarations */
typedef void (*StateHandler)(EdgeSensor *edgeSensor, ES_Event event);
typedef void (*ES_CommandHandler)(EdgeSensor *edgeSensor, const char *commandName, cJSON *payload);

/* ES_PredictiveModel Structure */
struct _ES_PredictiveModel {
    size_t size;
    uint8_t *buffer;
};

/* ES_Config Structure */
struct _ES_Config {
    uint32_t measurementIntervalMS;
};

/* ES_State Machine Structure */
struct _ES_StateMachine {
    ES_State state;
    StateHandler stateHandler;
};

/* Edge Sensor Structure */
struct edgeSensor {
    char *deviceName;

    int *pendingCommands;
    ES_CommandHandler commandHandler;
    SemaphoreHandle_t commandSemaphore;

    ES_StateMachine *stateMachine;
    ES_PredictiveModel *predictiveModel;
    ES_Config *config;

    cJSON *predictionLogJson;
};

/* Multilayer inference */
#define PREDICTION_ON_CLOUD(mask)    (((mask) & 0x04) == 0x04)
#define PREDICTION_ON_GATEWAY(mask)  (((mask) & 0x02) == 0x02)
#define PREDICTION_ON_DEVICE(mask)   (((mask) & 0x01) == 0x01)

/* Function Declarations */
void from_uint64_to_string(uint64_t timestamp, char *buffer);

void edge_sensor_measurement_payload(cJSON *jsonPayload, float measurement);
void edge_sensor_prediction_request_payload(cJSON *jsonPayload, char *request_timestamp, float measurement);
void edge_sensor_prediction_log_payload(cJSON *jsonPayload, char *source_layer, char *request_timestamp, char *response_timestamp, char *measurement, char *prediction);

void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_ready_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_working_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_stopped_handler(EdgeSensor *edgeSensor, ES_Event event);

void es_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *payload);
void es_state_machine_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *payload);
void es_predictive_model_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *payload);
void es_config_command_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *payload);

esp_err_t save_edge_sensor_to_nvs(EdgeSensor *edgeSensor);
esp_err_t set_nvs_edge_sensor_model_flag();
esp_err_t set_nvs_edge_sensor_config_flag();
esp_err_t set_nvs_edge_sensor_sleep_flag();
uint8_t get_nvs_edge_sensor_model_flag();
uint8_t get_nvs_edge_sensor_config_flag();
uint8_t get_nvs_edge_sensor_sleep_flag();
esp_err_t reset_nvs_edge_sensor();

esp_err_t edge_sensor_init(EdgeSensor *edgeSensor, char *deviceName, uint8_t sleep_flag, uint8_t model_flag, uint8_t config_flag);

float edge_sensor_measure(EdgeSensor *edgeSensor);

void edge_sensor_sleep(EdgeSensor *edgeSensor);

uint8_t edge_sensor_compute_inference_layers(EdgeSensor *edgeSensor);

uint64_t edge_sensor_timer_get_time();
void edge_sensor_save_timer_state(uint64_t delta);

#endif // EDGE_SENSOR_H
