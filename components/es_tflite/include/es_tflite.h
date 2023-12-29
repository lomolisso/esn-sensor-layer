#ifndef ES_TENSORFLOW_LITE_H_
#define ES_TENSORFLOW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "edge_sensor.h"

const float kXrange = 2.f * 3.14159265359f;
void es_tflite_init(ES_PredictiveModel *predictiveModel);
void es_tflite_predict(float *inputData, float *outputData);

#ifdef __cplusplus
}
#endif

#endif  // ES_TENSORFLOw_H_
