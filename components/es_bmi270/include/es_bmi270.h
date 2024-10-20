#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"


//=============================================================================
//                              Defines and global variables
//=============================================================================

// I2C pin configurations
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_BMI_NUM I2C_NUM_0
// I2C frequency (Hz)
#define I2C_MASTER_FREQ_HZ 100000

// I2C slave address
#define ESP_SLAVE_ADDR 0x68

// I2C write and read bit configurations
#define WRITE_BIT 0x0
#define READ_BIT 0x1

// I2C acknowledge (ACK) check and value configurations
#define ACK_CHECK_EN 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define DATA_READY_MASK 0x80
#define ANYMOTION_MASK 0b1000000

// Conversion factors for raw sensor data
#define MAX_INT_VALUE_SENSOR 32768.0
#define G_MS2 9.80665

#define ACCEL_RAW_TO_G (1.000 / MAX_INT_VALUE_SENSOR)
#define ACCEL_RAW_TO_MS2 (G_MS2 / MAX_INT_VALUE_SENSOR)
#define GYR_RAW_TO_RADS (3.14159265359 / 180.0) / MAX_INT_VALUE_SENSOR

// Config variables for sensor configuration and control
#define SENSOR_POWER_MODE 1 // 0: PM_LOW_POWER, 1: PM_NORMAL, 2: PM_PERFORMANCE, 3: NO_POWER_SETTING

#define SENSOR_ACC_RANGE 2 // 0: 2g, 1: 4g, 2: 8g, 3: 16g
#define SENSOR_GYR_RANGE 3 // 0: 2000°/s, 1: 1000°/s, 2: 500°/s, 3: 250°/s, 4: 125°/s

#define SENSOR_ACC_ODR 7    // 50Hz
#define SENSOR_GYR_ODR 7    // 50Hz

/* BMI 270 Sensor */
#define BMI270_SAMPLE_SIZE 6
#define BMI270_SEQUENCE_LENGTH 500

void es_bmi270_init(size_t configSize, uint8_t *configBuffer, uint8_t sleep_flag);
void es_bmi270_raw_measure(uint8_t *rawReadingBuffer);
void es_bmi270_measure(float **readingBuffer);

