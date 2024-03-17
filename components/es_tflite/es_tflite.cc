#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "es_tflite.h"
#include <string>

namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    TfLiteTensor *input = nullptr;
    TfLiteTensor *output = nullptr;
    int inference_count = 0;

    constexpr int kTensorArenaSize = 2 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];
}

extern "C" void es_tflite_init(ES_PredictiveModel *predictiveModel)
{
    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    if (predictiveModel == nullptr)
    {
        MicroPrintf("Model is null");
        return;
    }

    model = tflite::GetModel(predictiveModel->buffer);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.",
                    model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // Pull in only the operation implementations we need.
    static tflite::MicroMutableOpResolver<1> resolver;
    if (resolver.AddFullyConnected() != kTfLiteOk)
    {
        return;
    }

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed");
        return;
    }

    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);

    // Keep track of how many inferences we have performed.
    inference_count = 0;
    MicroPrintf("Model loaded successfully\n");
}

extern "C" void es_tflite_predict(float *inputData, float *outputData)
{
    // Calculate an x value to feed into the model. We compare the current
    // inference_count to the number of inferences per cycle to determine
    // our position within the range of possible x values the model was
    // trained on, and use this to calculate a value.

    float x = (*inputData) * kXrange;

    // Quantize the input from floating-point to integer
    int8_t x_quantized = x / input->params.scale + input->params.zero_point;

    // Place the quantized input in the model's input tensor
    input->data.int8[0] = x_quantized;

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
    {
        MicroPrintf("Invoke failed on x: %f\n",
                    static_cast<double>(x));
        return;
    }

    // Obtain the quantized output from model's output tensor
    int8_t y_quantized = output->data.int8[0];

    // Dequantize the output from integer to floating-point
    float y = (y_quantized - output->params.zero_point) * output->params.scale;
    *outputData = y;
}