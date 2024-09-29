#ifndef EDGE_SENSOR_H
#define EDGE_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "cJSON.h"
#include <stdint.h>

typedef float float32_t;

/* Adaptive Inference */
#define ADAPTIVE_INFERENCE 0
#define SIGMA(Mt) (__builtin_popcount(Mt))
#define ABNORMAL_PREDICTION_THRESHOLD 8
#define IS_PREDICTION_ABNORMAL(p) (p == 2 || p == 3)

#define PREDICTION_HISTORY_LENGTH 16
#define UPDATE_PREDICTION_HISTORY(Mt, p) (Mt = ((Mt << 1) | p) & ((1 << PREDICTION_HISTORY_LENGTH) - 1))
#define UPDATE_PREDICTION_COUNTER(ut) (ut = (ut + 1) % PREDICTION_HISTORY_LENGTH)

#define BATTERY_CAPACITY 10000              //mAh
#define REMAINING_BATTERY_THRESHOLD 20      //mAh
#define NORMAL_CYCLE_CONSUMPTION 1          //mAh
#define INFERENCE_CYCLE_CONSUMPTION 3       //mAh
#define IS_LOW_BATTERY(remainingBattery) (remainingBattery < REMAINING_BATTERY_THRESHOLD)

#define BMI270_SAMPLE_SIZE 6
#define BMI270_SEQUENCE_LENGTH 25

/* NVS */
#define EDGE_SENSOR_PARTITION "edge_sensor_data"
#define EDGE_SENSOR_PARTITION_NAMESPACE "edge_sensor_ns"

/* Edge Sensor Properties */
#define ES_METHOD_SET "set"
#define ES_METHOD_GET "get"

// Sensor state property (str)
#define ES_PROPERTY_SENSOR_STATE "sensor-state"

// Sensor model property (JSON)
#define ES_PROPERTY_SENSOR_MODEL "sensor-model"
#define SENSOR_MODEL_B64_FIELD "tf_model_b64"           // char *
#define SENSOR_MODEL_BYTESIZE_FIELD "tf_model_bytesize" // int32_t

// Inference layer property (int)
#define ES_PROPERTY_INFERENCE_LAYER "inference-layer"

// Sensor config property (JSON)
#define ES_PROPERTY_SENSOR_CONFIG "sensor-config"
#define SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_FIELD "sleep_interval_ms" // int32_t

/* Edge Sensor Exports */

// Sensor data export (JSON)
#define ES_EXPORT_SENSOR_DATA "sensor-data"
#define SENSOR_DATA_LOW_BATTERY_FIELD "low_battery" // bool

#define SENSOR_DATA_READING_FIELD "reading"       // JSON
#define SENSOR_DATA_READING_VALUES_FIELD "values" // array of array of float32_t

#define SENSOR_DATA_INFERENCE_DESCRIPTOR_FIELD "inference_descriptor"            // JSON
#define SENSOR_DATA_INFERENCE_DESCRIPTOR_INFERENCE_LAYER_FIELD "inference_layer" // int32_t
#define SENSOR_DATA_INFERENCE_DESCRIPTOR_PREDICTION_FIELD "prediction"           // int32_t
#define SENSOR_DATA_INFERENCE_DESCRIPTOR_SEND_TIMESTAMP_FIELD "send_timestamp"   // char *
#define SENSOR_DATA_INFERENCE_DESCRIPTOR_RECV_TIMESTAMP_FIELD "recv_timestamp"   // char *

/* Forward Declarations */
typedef struct edgeSensor EdgeSensor;
typedef struct _ES_StateMachine ES_StateMachine;
typedef struct _ES_PredictiveModel ES_PredictiveModel;
typedef struct _ES_Config ES_Config;
typedef struct _ES_BMI270 ES_BMI270;

/* Edge Sensor States */
typedef enum _ES_State
{
    STATE_INITIAL,
    STATE_UNLOCKED,
    STATE_LOCKED,
    STATE_WORKING,
    STATE_IDLE,
    STATE_ERROR
} ES_State;

#define FROM_STRING_STATE_TO_ENUM(state) ( \
    strcmp(state, "initial") == 0 ? STATE_INITIAL : \
    strcmp(state, "unlocked") == 0 ? STATE_UNLOCKED : \
    strcmp(state, "locked") == 0     ? STATE_LOCKED : \
    strcmp(state, "working") == 0    ? STATE_WORKING : \
    strcmp(state, "idle") == 0       ? STATE_IDLE : \
    strcmp(state, "error") == 0      ? STATE_ERROR : \
    STATE_INITIAL)

#define FROM_ENUM_STATE_TO_STRING(state) ( \
    state == STATE_INITIAL ? "initial" : \
    state == STATE_UNLOCKED ? "unlocked" : \
    state == STATE_LOCKED ? "locked" : \
    state == STATE_WORKING ? "working" : \
    state == STATE_IDLE ? "idle" : \
    state == STATE_ERROR ? "error" : \
    "initial")


/* Edge Sensor Events */
typedef enum _ES_Event
{
    EVENT_ES_BOOTED_UP,                   // intial -> unlocked
    EVENT_ES_LOAD_STATE_LOCKED_FROM_NVS,  // unlocked -> locked
    EVENT_ES_LOAD_STATE_WORKING_FROM_NVS, // unlocked -> working
    EVENT_ES_LOAD_STATE_IDLE_FROM_NVS,    // unlocked -> idle
    EVENT_ES_LOAD_STATE_ERROR_FROM_NVS,   // unlocked -> error
    EVENT_CMD_SET_STATE_LOCKED_RECV,      // unlocked -> locked
    EVENT_CMD_SET_STATE_UNLOCKED_RECV,    // locked -> unlocked
    EVENT_CMD_SET_STATE_WORKING_RECV,     // locked -> working
    EVENT_CMD_SET_STATE_IDLE_RECV,        // working -> idle
    EVENT_CMD_SET_STATE_ERROR_RECV,       // * -> error
    EVENT_CMD_SET_STATE_INITIAL_RECV,     // * -> initial
} ES_Event;

#define FROM_ENUM_EVENT_TO_STRING(event) ( \
    event == EVENT_ES_BOOTED_UP ? "EVENT_ES_BOOTED_UP" : \
    event == EVENT_ES_LOAD_STATE_LOCKED_FROM_NVS ? "EVENT_ES_LOAD_STATE_LOCKED_FROM_NVS" : \
    event == EVENT_ES_LOAD_STATE_WORKING_FROM_NVS ? "EVENT_ES_LOAD_STATE_WORKING_FROM_NVS" : \
    event == EVENT_ES_LOAD_STATE_IDLE_FROM_NVS ? "EVENT_ES_LOAD_STATE_IDLE_FROM_NVS" : \
    event == EVENT_ES_LOAD_STATE_ERROR_FROM_NVS ? "EVENT_ES_LOAD_STATE_ERROR_FROM_NVS" : \
    event == EVENT_CMD_SET_STATE_LOCKED_RECV ? "EVENT_CMD_SET_STATE_LOCKED_RECV" : \
    event == EVENT_CMD_SET_STATE_UNLOCKED_RECV ? "EVENT_CMD_SET_STATE_UNLOCKED_RECV" : \
    event == EVENT_CMD_SET_STATE_WORKING_RECV ? "EVENT_CMD_SET_STATE_WORKING_RECV" : \
    event == EVENT_CMD_SET_STATE_IDLE_RECV ? "EVENT_CMD_SET_STATE_IDLE_RECV" : \
    event == EVENT_CMD_SET_STATE_ERROR_RECV ? "EVENT_CMD_SET_STATE_ERROR_RECV" : \
    event == EVENT_CMD_SET_STATE_INITIAL_RECV ? "EVENT_CMD_SET_STATE_INITIAL_RECV" : \
    "UNKNOWN_EVENT")

void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_unlocked_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_locked_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_working_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_idle_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_error_handler(EdgeSensor *edgeSensor, ES_Event event);

/* Function Pointer Types using Forward Declarations */
typedef void (*StateHandler)(EdgeSensor *edgeSensor, ES_Event event);
typedef cJSON *(*ES_CommandHandler)(EdgeSensor *edgeSensor, const char *propertyName, const char *cmdMethod, cJSON *payload);

/* Inference Layer Enum */
typedef enum _ES_InferenceLayer
{
    INFERENCE_LAYER_SENSOR,
    INFERENCE_LAYER_GATEWAY,
    INFERENCE_LAYER_CLOUD
} ES_InferenceLayer;

/* ES_PredictiveModel Structure */
struct _ES_PredictiveModel
{
    size_t size;
    uint8_t *buffer;
    uint32_t predictionHistory;
    uint32_t predictionCounter;
};

/* ES_Config Structure */
struct _ES_Config
{
    uint32_t measurementIntervalMS;
};

/* ES_State Machine Structure */
struct _ES_StateMachine
{
    ES_State state;
    StateHandler stateHandler;
};

/* ES_BMI270 */
struct _ES_BMI270
{
    uint8_t *configBuffer;
    uint32_t configSize;
    float32_t **readingBuffer;
};

/* Edge Sensor Structure */
struct edgeSensor
{
    char *deviceName;
    uint32_t remainingBattery;

    
    ES_Config *config;
    ES_InferenceLayer inferenceLayer;
    ES_PredictiveModel *predictiveModel;
    ES_BMI270 *bmi270;
    
    ES_CommandHandler commandHandler;
    ES_StateMachine *stateMachine;

    SemaphoreHandle_t mutexSemaphore;
};

/* Utility Function Declarations */
void from_uint64_to_string(uint64_t timestamp, char *buffer);

/* Build JSON Payload Functions */
typedef struct sensorDataExport
{
    uint8_t lowBattery;
    float32_t **reading;
    uint8_t inferenceLayer;
    uint64_t *sendTimestamp; // uint64_t || NULL
    uint64_t *recvTimestamp; // uint64_t || NULL
    uint8_t *prediction;     // uint8_t || NULL
} SensorDataExport;
void build_export_sensor_data_payload(cJSON *jsonPayload, SensorDataExport *sensorDataExport);

/* Command Handler Function Declarations */
cJSON *es_command_handler(EdgeSensor *edgeSensor, const char *propertyName, const char *cmdMethod, cJSON *payload);

/* NVS Function Declarations */
#define NVS_INF_FLAG_KEY "inf_flag"
#define NVS_MODEL_FLAG_KEY "model_flag"
#define NVS_CONFIG_FLAG_KEY "config_flag"
#define NVS_SLEEP_FLAG_KEY "sleep_flag"
#define NVS_BMI270_CONFIG_FLAG_KEY "bmi270_flag"

esp_err_t set_nvs_edge_sensor_inf_flag(uint8_t inf_flag);
esp_err_t set_nvs_edge_sensor_model_flag(uint8_t model_flag);
esp_err_t set_nvs_edge_sensor_config_flag(uint8_t config_flag);
esp_err_t set_nvs_edge_sensor_sleep_flag(uint8_t sleep_flag);

uint8_t get_nvs_edge_sensor_inf_flag();
uint8_t get_nvs_edge_sensor_model_flag();
uint8_t get_nvs_edge_sensor_config_flag();
uint8_t get_nvs_edge_sensor_sleep_flag();
uint8_t get_nvs_bmi270_config_flag();

#define NVS_INFERENCE_LAYER_SENSOR_KEY "inference_layer"
#define NVS_SENSOR_BATTERY_MANAGER_LOW_BATTERY_KEY "low_battery"
#define NVS_SENSOR_REMAINING_BATTERY_KEY "rem_batt"
#define NVS_SENSOR_MODEL_B64_KEY "model_bytes"
#define NVS_SENSOR_MODEL_BYTESIZE_KEY "model_size"
#define NVS_SENSOR_CONFIG_MEASUREMENT_INTERVAL_MS_KEY "sleep_t"
#define NVS_SENSOR_STATE_KEY "sensor_state"
#define NVS_BMI270_CONFIG_KEY "bmi270_conf"
#define NVS_BMI270_CONFIG_SIZE_KEY "bmi270_size"

esp_err_t save_edge_sensor_to_nvs(EdgeSensor *edgeSensor);
esp_err_t reset_nvs_edge_sensor();

/* Edge Sensor Structure Function Declarations */
esp_err_t edge_sensor_init(EdgeSensor *edgeSensor, char *deviceName);

void edge_sensor_measure(EdgeSensor *edgeSensor);
void edge_sensor_sleep(EdgeSensor *edgeSensor);
void edge_sensor_update_inference_layer(EdgeSensor *edgeSensor);

#endif // EDGE_SENSOR_H
