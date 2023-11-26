#ifndef EDGE_SENSOR_H
#define EDGE_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "cJSON.h"

// Forward Declarations
typedef struct edgeSensor EdgeSensor;
typedef struct _ES_StateMachine ES_StateMachine;
typedef struct _ES_Config ES_Config;

// Edge Sensor States
typedef enum _ES_State {
    STATE_INITIAL,
    STATE_CONFIGURED,
    STATE_WORKING,
    STATE_STOPPED,
} ES_State;

// Edge Sensor Events
typedef enum _ES_Event {
    EVENT_CONFIG_COMMAND_RECEIVED,
    EVENT_START_COMMAND_RECEIVED,
    EVENT_STOP_COMMAND_RECEIVED,
    EVENT_RESET_COMMAND_RECEIVED,
} ES_Event;

// Function Pointer Types using Forward Declarations
typedef void (*StateHandler)(EdgeSensor *edgeSensor, ES_Event event);
typedef void (*ES_CommandHandler)(EdgeSensor *edgeSensor, const char *commandName, const char *commandMethod, const char *payload);

// ES_Config Structure
struct _ES_Config {
    uint32_t measurementIntervalMS;
    uint8_t runPrediction;
    char *predictiveModel;
};

// ES_State Machine Structure
struct _ES_StateMachine {
    ES_State state;
    StateHandler stateHandler;
    SemaphoreHandle_t stateMutex;
    SemaphoreHandle_t stateSemaphore;
};

// ES_PredictiveModel Structure
struct _ES_PredictiveModel {
    char *modelPath;
};

// Edge Sensor Structure
struct edgeSensor {
    char *deviceName;
    ES_CommandHandler commandHandler;
    ES_Config *config;
    ES_StateMachine *stateMachine;
};

// Function Declarations
void state_initial_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_configured_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_working_handler(EdgeSensor *edgeSensor, ES_Event event);
void state_stopped_handler(EdgeSensor *edgeSensor, ES_Event event);

void es_command_handler(EdgeSensor *edgeSensor, const char *commandName, const char *commandMethod, const char *payload);
void es_config_handler(EdgeSensor *edgeSensor, const char *commandName, cJSON *jsonConfigField);

uint8_t is_edge_sensor_configured();
esp_err_t reset_edge_sensor_config_from_nvs();
esp_err_t save_edge_sensor_to_nvs(EdgeSensor *edgeSensor);
esp_err_t load_edge_sensor_from_nvs(EdgeSensor *edgeSensor, ES_Config *config, ES_StateMachine *stateMachine, char *deviceName);

void edge_sensor_init(EdgeSensor *edgeSensor, ES_Config *config, ES_StateMachine *stateMachine, char *deviceName);

float edge_sensor_measure(EdgeSensor *edgeSensor);
float edge_sensor_predict(EdgeSensor *edgeSensor, float prediction);

void edge_sensor_sleep(EdgeSensor *edgeSensor);

#endif // EDGE_SENSOR_H
