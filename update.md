# Updates to ESP32 Code

## States:
- `initial`
- `unlocked`
- `locked`
- `working`
- `idle`
- `error`

## Events:
1. `ES_BOOTED_UP:` Event emitted after booting up, before loading from NVS.
2. `ES_LOAD_STATE_LOCKED_FROM_NVS:` Event emitted after loading the `locked` state from NVS.
3. `ES_LOAD_STATE_WORKING_FROM_NVS:` Event emitted after loading the `working` state from NVS.
4. `ES_LOAD_STATE_IDLE_FROM_NVS:` Event emitted after loading the `idle` state from NVS.
5. `ES_LOAD_STATE_ERROR_FROM_NVS:` Event emitted after loading the `error` state from NVS.

6. `ES_SET_STATE_LOCKED_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `locked`.
7. `ES_SET_STATE_UNLOCKED_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `unlocked`.
8. `ES_SET_STATE_WORKING_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `working`.
9. `ES_SET_STATE_IDLE_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `idle`.
10. `ES_SET_STATE_ERROR_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `error`.
11. `ES_SET_STATE_INITIAL_CMD_RECV:` Event emitted after receiving a SET command for the property `sensor-state` with value `initial`.

## Commands:
Commands are sent to the device to either SET or GET a property. The command topic is of the form:
```command/<device_name>/<property_name>/<method>/<cmd_uuid>```

GET commands require a response with a topic of the form:
```response/<device_name>/<property_name>/get/<cmd_uuid>```

The following properties are available for the device:
### `sensor-state:` 
The state of the device, supports both SET and GET commands.
```json
{
    "state": "<state-value: str>" 
}
```
where `<state-value>` can be one of the states mentioned above.

### `inference-layer`
The inference layer of the sensor, i.e. the location where the inference takes place. This property supports both SET and GET commands.
```json
{
    "inference-layer": "<state-value: int>"
}
```
where `<state-value>` can be one of the following:
```
0: `SENSOR`
1: `GATEWAY`
2: `CLOUD`
```

### `sensor-config`
The configuration of the sensor, i.e. the parameters which domain certain aspects of the sensor behaviour. This property supports both SET and GET commands.

```json
{
    "sensor-config": {
        "sleep_interval_ms": "<sleep-interval-ms-value: int>",
    }
}
```
where `<sleep-interval-ms-value>` is the time in milliseconds for which the sensor sleeps before waking up again.

### `sensor-model`
The model of the sensor, i.e. the ML model used for inference. This property supports **only** SET commands as the model is not expected to change during runtime.

```json
{
    "sensor-model": {
        "tf_model_b64": "<tf-model-b64-value: str>",
        "tf_model_bytesize": "<tf-model-bytesize-value: int>"
    }
}
```
where `<tf-model-b64-value>` is the gzipped tensorflow lite model encoded as a base64 string, `<tf-model-bytesize-value>` is the size of the model in bytes.

## Exports:
Exports are sent by the device to the broker. The export topic is of the form:
```export/<device_name>/<export_name>```

The following exports are available for the device:

### `sensor-data`
This export is composed of both sensor metadata such as if the device has `low_battery`, the `inference_layer` and the time of the export, as well as the actual reading collected by the sensor. The export is of the form:
```json
{
    "low_battery": "<low-battery-value: bool>",
    "reading": {
        "values": "<values-value: list[list[float32]]>"
    },
    "inference_descriptor": {
        "inference_layer": "<inference-layer-value: int>",
        "prediction": "<prediction-value: int | null>",
        "send_timestamp": "<send-timestamp-value: int>",
        "recv_timestamp": "<recv-timestamp-value: int | null>"
    }
}
```
where `<low-battery-value>` is a boolean indicating if the device has low battery, `<values-value>` is a list of lists of float32 values representing the sensor readings, `<inference-layer-value>` is the layer where the inference will take place. If the inference layer is not `SENSOR`, the `prediction` and `recv_timestamp` will be `null`. 
