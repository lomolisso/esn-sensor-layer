#ifndef ES_TENSORFLOW_LITE_H_
#define ES_TENSORFLOW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef float float32_t;
typedef unsigned char uint8_t;

#define TFLITE_OP_RESOLVER_SIZE 70
#define SEQUENCE_LENGTH 25
#define SAMPLE_SIZE 6

void es_tflite_init(uint8_t *predictiveModelBuffer);
void es_tflite_predict(float32_t **inputData, uint8_t *outputData);



#ifdef __cplusplus
}
#endif

#endif  // ES_TENSORFLOw_H_
