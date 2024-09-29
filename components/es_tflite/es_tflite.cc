#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "es_tflite.h"

#define ADD_OPERATION_TO_RESOLVER(op) \
    if (resolver.op() != kTfLiteOk) { \
        MicroPrintf(TAG, #op " resolver failed"); \
        return; \
    }


namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    tflite::MicroMutableOpResolver<TFLITE_OP_RESOLVER_SIZE> resolver;
    
    TfLiteTensor *input = nullptr;
    TfLiteTensor *output = nullptr;

    constexpr int kTensorArenaSize = 16 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];

    static const char *TAG = "ES_TFLM";
}

extern "C" void es_tflite_init(uint8_t *predictiveModelBuffer)
{
    if (predictiveModelBuffer == nullptr)
    {
        MicroPrintf("Predictive model is NULL.\n");
        return;
    }

    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    model = tflite::GetModel(predictiveModelBuffer);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf("TFLITE_SCHEMA_VERSION mismatch.\n");
        MicroPrintf("Expected: %d\n", TFLITE_SCHEMA_VERSION);
        MicroPrintf("Received: %d\n", model->version());
        return;
    }

    ADD_OPERATION_TO_RESOLVER(AddQuantize)
    ADD_OPERATION_TO_RESOLVER(AddDequantize)
    ADD_OPERATION_TO_RESOLVER(AddConv2D)
    ADD_OPERATION_TO_RESOLVER(AddDepthwiseConv2D)
    ADD_OPERATION_TO_RESOLVER(AddMaxPool2D)
    ADD_OPERATION_TO_RESOLVER(AddReshape)
    ADD_OPERATION_TO_RESOLVER(AddFullyConnected)
    ADD_OPERATION_TO_RESOLVER(AddSoftmax)
    ADD_OPERATION_TO_RESOLVER(AddRelu)
    ADD_OPERATION_TO_RESOLVER(AddExpandDims)

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        MicroPrintf("AllocateTensors() failed.\n");
        return;
    }

    // Obtain pointers to the model's input and output tensors.
    input = interpreter->input(0);
    output = interpreter->output(0);

    MicroPrintf("Model loaded successfully.\n");
}


void _preprocess_input(float **inputData)
{
    /*
    This function quantizes the input sequence of samples from float to uint8.
    */
    for (int i = 0; i < SEQUENCE_LENGTH; i++)
    {
        for (int j = 0; j < SAMPLE_SIZE; j++)
        {
            uint8_t x_quantized = inputData[i][j] / input->params.scale + input->params.zero_point;
            input->data.uint8[i * SAMPLE_SIZE + j] = x_quantized;
        }
    }
}

void _postprocess_output(uint8_t *outputData)
{
    /*
    This applies the argmax function to the output data to obtain the most likely class.
    */
    uint8_t max_index = 0;
    uint8_t max_value = 0;
    uint8_t numClasses = output->dims->data[1];
    MicroPrintf("Number of classes: %d\n", numClasses);

    for (int i = 1; i < numClasses; i++)
    {
        if (output->data.uint8[i] > max_value)
        {
            max_value = output->data.uint8[i];
            max_index = i;
        }
    }
    *outputData = max_index;
}

extern "C" void es_tflite_predict(float32_t **inputData, uint8_t *outputData)
{   
    /*
    Input is a sequence of SEQUENCE_LENGTH samples, each with SAMPLE_SIZE floating-point values.
    The model went through full integer quantization, so the input and output tensors are expected to be uint8.
    Therefore, the input data must be quantized to uint8 before being placed in the model's input tensor.
    The output is a list of probabilities for each class, and the most likely class is obtained by applying the argmax function.
    Consequently, the output data does not need to be dequantized, i.e. Argmax can be applied directly to the output tensor.
    */

    // Step 1: Quantize the input data interating over the sequence of samples
    _preprocess_input(inputData);

    // Step 2: Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
    {
        MicroPrintf("Invoke() failed.\n");
        return;
    }

    // Step 3: Dequantize the output data
    _postprocess_output(outputData);
}