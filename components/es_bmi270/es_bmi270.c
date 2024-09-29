#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "math.h"
#include "esp_err.h"

#include "es_bmi270.h"

static const char *TAG = "ES_BMI270";

// Register addresses for sensor configuration and control
uint8_t REG_ID = 0x00;
uint8_t REG_PWR_CTRL = 0x7D;
uint8_t REG_ACC_CONF = 0x40;
uint8_t REG_ACC_RANGE = 0x41;
uint8_t REG_GYR_CONF = 0x42;
uint8_t REG_GYR_RANGE = 0x43;
uint8_t REG_PWR_CONF = 0x7C;
uint8_t REG_PWR_CONF_ADVPOWERSAVE = 0x7C;
uint8_t REG_INIT_CTRL = 0x59;
uint8_t REG_INIT_DATA = 0x5E;
uint8_t REG_INTERNAL_STATUS = 0x21;
uint8_t REG_INT_STATUS_0 = 0x1C;
uint8_t REG_INT_STATUS = 0x03;
uint8_t REG_DATA_8 = 0x0C;
uint8_t REG_ANYMO_1 = 0x3C;  // Registro para configurar anymotion
uint8_t REG_ANYMO_2 = 0x3E;  // Registro para activar anymotion

esp_err_t bmi_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd,
                   size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bmi_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr,
                    size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bmi_init(void) {
    int i2c_master_port = I2C_BMI_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

//=============================================================================
//                              Predefined functions
//=============================================================================

esp_err_t chip_id(void) {
    uint8_t tmp;
    esp_err_t ret;

    ret = bmi_read(I2C_BMI_NUM, &REG_ID, &tmp, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID: %d\n\n", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Value of CHIP_ID: %2X \n\n", tmp);
    if (tmp != 0x24) {
        ESP_LOGE(TAG, "Chip not recognized. \nCHIP ID: %2x\n\n", tmp);  // %2X
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t softreset(void) {
    uint8_t reg_softreset = 0x7E, val_softreset = 0xB6;

    esp_err_t ret = bmi_write(I2C_BMI_NUM, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return ret;
}

esp_err_t initialization(size_t bmi270_config_size, uint8_t *bmi270_config_file) {
    uint8_t val_pwr_conf_advpowersave = 0x00;
    uint8_t val_init_ctrl = 0x00;
    uint8_t val_init_ctrl2 = 0x01;

    ESP_LOGI(TAG, "Initializing ...\n");

    bmi_write(I2C_BMI_NUM, &REG_PWR_CONF_ADVPOWERSAVE,
              &val_pwr_conf_advpowersave, 1);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    esp_err_t ret = bmi_write(I2C_BMI_NUM, &REG_INIT_CTRL, &val_init_ctrl, 1);

    ret = bmi_write(I2C_BMI_NUM, &REG_INIT_DATA, (uint8_t *)bmi270_config_file,
                    bmi270_config_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "\nErorr loding config_file\n");
    } else {
        ESP_LOGI(TAG, "\nSuccesfully loaded config_file\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ret = bmi_write(I2C_BMI_NUM, &REG_INIT_CTRL, &val_init_ctrl2, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "\nError in initialization: %s \n", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "\nInitialization: OK!\n\n");
    }

    return ret;
}

void internal_status(void) {
    uint8_t tmp;

    bmi_read(I2C_BMI_NUM, &REG_INTERNAL_STATUS, &tmp, 1);
    ESP_LOGI(TAG, "Internal Status: %2X\n\n", tmp);
}

void check_initialization(void) {
    uint8_t tmp;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    bmi_read(I2C_BMI_NUM, &REG_INTERNAL_STATUS, &tmp, 1);
    ESP_LOGI(TAG, "Init_status.0: %x \n", (tmp & 0b00001111));
    if ((tmp & 0b00001111) == 1) {
        ESP_LOGI(TAG, "Initialization check: OK\n\n");
    } else {
        ESP_LOGE(TAG, "Initialization failed\n\n");
        exit(EXIT_SUCCESS);
    }
}
void intToBinaryString(int n, char *binaryString, int size) {
    if (size < 33) return;  // Check if the array is large enough

    binaryString[32] = '\0';  // Null-terminate the string at the end

    for (int i = 31; i >= 0; --i, n >>= 1) {
        binaryString[i] = (n & 1) + '0';  // Convert bit to '0' or '1' character
    }
}

void printBin(char *help_tag, int number) {
    char binaryString[33];
    intToBinaryString(number, binaryString, 33);
    ESP_LOGI(TAG, "%s: %s\n", help_tag, binaryString);
}
esp_err_t set_check(i2c_port_t i2c_num, uint8_t *reg, uint8_t *value,
                    size_t size, char *help_str) {
    esp_err_t ret = ESP_OK;
    ret = bmi_write(I2C_BMI_NUM, reg, value, size);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write %s: %d\n\n", help_str, ret);
        return ret;
    }

    uint8_t *tmp = malloc(size);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    ret = bmi_read(I2C_BMI_NUM, reg, tmp, size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %s: %d\n\n", help_str, ret);
    }
    if (size == 2) {
        printBin(help_str, *((uint16_t *)value));
        printBin(help_str, *((uint16_t *)tmp));
    }
    if (size == 1) {
        printBin(help_str, *(value));
        printBin(help_str, *(tmp));
    }
    for (int i = 0; i < size; i++) {
        if (tmp[i] != value[i]) {
            ESP_LOGE(TAG, "Failed to set %s: %d\n\n", help_str, ret);
            return ESP_FAIL;
        }
    }
    free(tmp);
    ESP_LOGI(TAG, "Set %s: %d\n\n", help_str, *value);
    return ret;
}

esp_err_t read_check(i2c_port_t i2c_num, uint8_t *reg, uint8_t *value,
                     size_t size, char *help_str) {
    esp_err_t ret = ESP_OK;

    ret = bmi_read(I2C_BMI_NUM, reg, value, size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %s: %d\n\n", help_str, ret);
    } else {
        ESP_LOGI(TAG, "Read %s: %d\n\n", help_str, *value);
    }
    return ret;
}

//=============================================================================
//                             POWER MODES
//=============================================================================

esp_err_t set_power_mode(uint8_t mode) {
    ESP_LOGI(TAG, "Setting power mode: ");
    uint8_t val_pwr_ctrl, val_acc_conf, val_gyr_conf, val_pwr_conf;
    val_pwr_ctrl = val_acc_conf = val_gyr_conf = val_pwr_conf = 0x00;
    esp_err_t ret = ESP_OK;
    switch (mode) {
        case 0:
            ESP_LOGI(TAG, "PM_LOW_POWER\n");
            val_pwr_ctrl = 0x04;
            val_acc_conf = 0x17;
            val_pwr_conf = 0x03;
            break;
        case 1:
            ESP_LOGI(TAG, "PM_NORMAL\n");
            val_pwr_ctrl = 0x0E;
            val_acc_conf = 0xA8;
            val_gyr_conf = 0xA9;
            val_pwr_conf = 0x02;
            break;
        case 2:
            ESP_LOGI(TAG, "PM_PERFORMANCE\n");
            val_pwr_ctrl = 0x0E;
            val_acc_conf = 0xA8;
            val_gyr_conf = 0xE9;
            val_pwr_conf = 0x02;
            break;
        case 3:
            ESP_LOGI(TAG, "No change\n");
            return ret;
        default:
            ESP_LOGI(TAG, "ERROR\n");
            ret = ESP_FAIL;
            break;
    }
    if (val_pwr_ctrl)
        set_check(I2C_BMI_NUM, &REG_PWR_CTRL, &val_pwr_ctrl, 1, "POWER_MODE");
    if (val_acc_conf)
        set_check(I2C_BMI_NUM, &REG_ACC_CONF, &val_acc_conf, 1, "POWER_MODE");
    if (val_gyr_conf)
        set_check(I2C_BMI_NUM, &REG_GYR_CONF, &val_gyr_conf, 1, "POWER_MODE");
    if (val_pwr_conf)
        set_check(I2C_BMI_NUM, &REG_PWR_CONF, &val_pwr_conf, 1, "POWER_MODE");
    return ret;
}

//=============================================================================
//                             ACC CONF AND RANGE
//=============================================================================

esp_err_t set_acc_odr(uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t cur;
    uint8_t mask = 0b00001111;

    ret = read_check(I2C_BMI_NUM, &REG_ACC_CONF, &cur, 1, "ACC_CONF: odr");

    cur &= ~mask;
    cur |= value;

    ret = set_check(I2C_BMI_NUM, &REG_ACC_CONF, &cur, 1, "ACC_CONF: odr");
    return ret;
}

esp_err_t set_acc_range(uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t mask = 0b00000011;
    uint8_t cur;
    ret = read_check(I2C_BMI_NUM, &REG_ACC_RANGE, &cur, 1, "ACC_RANGE: range");

    cur &= ~mask;
    cur |= value;

    ret = set_check(I2C_BMI_NUM, &REG_ACC_RANGE, &cur, 1, "ACC_RANGE: range");
    return ret;
}
//=============================================================================
//                             GYR CONF AND RANGE
//=============================================================================
esp_err_t set_gyr_odr(uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t mask = 0b00001111;
    uint8_t cur;
    ret = read_check(I2C_BMI_NUM, &REG_GYR_CONF, &cur, 1, "GYR_CONF: odr");

    cur &= ~mask;
    cur |= value;

    ret = set_check(I2C_BMI_NUM, &REG_GYR_CONF, &cur, 1, "GYR_CONF: odr");
    return ret;
}

esp_err_t set_gyr_range(uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t mask = 0b00000111;
    uint8_t cur;
    ret = read_check(I2C_BMI_NUM, &REG_GYR_RANGE, &cur, 1, "GYR_CONF: range");

    cur &= ~mask;
    cur |= value;

    ret = set_check(I2C_BMI_NUM, &REG_GYR_RANGE, &cur, 1, "GYR_CONF: range");
    return ret;
}

//=============================================================================
//                             ANYMOTION
//=============================================================================

esp_err_t anymotion_setup_anymo_2(uint16_t threshold) {
    esp_err_t ret = ESP_OK;
    uint16_t cur = 0;

    cur |= (1 << 15);     // enable anymotion
    cur |= (0x07 << 11);  // set ouput to 6 bit of REG_INT_STATUS_0
    cur |= threshold;     // bit 10..0

    ret = set_check(I2C_BMI_NUM, &REG_ANYMO_2, (uint8_t *)&cur, 2,
                    "ANYMO_2: enable");
    return ret;
}

esp_err_t anymotion_setup_anymo_1(uint16_t duration) {
    esp_err_t ret = ESP_OK;
    uint16_t cur = 0;

    cur |= duration;
    // enable all 3 axis
    cur |= (1 << 15) | (1 << 14) | (1 << 13);

    ret = set_check(I2C_BMI_NUM, &REG_ANYMO_1, (uint8_t *)&cur, 1,
                    "ANYMO_1: axis");
    return ret;
}

esp_err_t set_thresh_dur(uint8_t threshold, uint8_t duration) {
    esp_err_t ret = ESP_OK;
    uint8_t cur = 0;

    cur |= threshold;

    ret = set_check(I2C_BMI_NUM, &REG_ANYMO_2, (uint8_t *)&cur, 1,
                    "ANYMO_2: threshold");

    cur = 0;
    cur |= duration;

    ret = set_check(I2C_BMI_NUM, &REG_ANYMO_1, (uint8_t *)&cur, 1,
                    "ANYMO_1: duration");
    return ret;
}

esp_err_t setup_anymotion(uint8_t mode) {
    esp_err_t ret = ESP_OK;
    ESP_ERROR_CHECK(anymotion_setup_anymo_2(0xAA));

    ESP_ERROR_CHECK(anymotion_setup_anymo_1(5));

    // set_thresh_dur(0xAA, 5);
    uint16_t anymo1, anymo2;
    read_check(I2C_BMI_NUM, &REG_ANYMO_1, (uint8_t *)&anymo1, 2, "ANYMO_1");
    read_check(I2C_BMI_NUM, &REG_ANYMO_2, (uint8_t *)&anymo2, 2, "ANYMO_2");
    printBin("ANYMO_1", anymo1);
    printBin("ANYMO_2", anymo2);

    // sleep
    return ret;
}

//=============================================================================
//                             MAIN LOOP
//=============================================================================

float accel_raw_to_ms2(int16_t raw) {
    return (pow(2, SENSOR_ACC_RANGE + 1) * ACCEL_RAW_TO_MS2) * raw;
}

float accel_raw_to_g(int16_t raw) {
    return (pow(2, SENSOR_ACC_RANGE + 1) * ACCEL_RAW_TO_G) * raw;
}
float gyr_raw_to_rads(int16_t raw) {
    int values[] = {2000.0, 1000.0, 500.0, 250.0, 125.0};
    return values[SENSOR_GYR_RANGE] * GYR_RAW_TO_RADS * raw;
}
uint16_t combine_bytes(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | lsb;
}

int is_data_ready() {
    uint8_t status;
    bmi_read(I2C_BMI_NUM, &REG_INT_STATUS, &status, 1);
    return (status & DATA_READY_MASK) == DATA_READY_MASK;
}

int anymotion_detected() {
    uint8_t status;
    bmi_read(I2C_BMI_NUM, &REG_INT_STATUS_0, &status, 1);

    int res = (status & ANYMOTION_MASK) == ANYMOTION_MASK;
    if (res) {
        ESP_LOGI(TAG, "Anymotion detected\n");
    }
    return res;
}

void print_data() {
    esp_err_t ret = ESP_OK;
    uint8_t data_data8[12];
    uint16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    ret = bmi_read(I2C_BMI_NUM, &REG_DATA_8, (uint8_t *)data_data8,
                   sizeof(data_data8));
    acc_x = combine_bytes(data_data8[1], data_data8[0]);
    acc_y = combine_bytes(data_data8[3], data_data8[2]);
    acc_z = combine_bytes(data_data8[5], data_data8[4]);
    gyr_x = combine_bytes(data_data8[7], data_data8[6]);
    gyr_y = combine_bytes(data_data8[9], data_data8[8]);
    gyr_z = combine_bytes(data_data8[11], data_data8[10]);
    ESP_LOGI(TAG,
             "AcC: (%.2f, %.2f, %.2f) m/s² (%.2f, %.2f, %.2f) g |  Gy: (%.2f, "
             "%.2f, %.2f) rad/s",
             accel_raw_to_ms2((int16_t)acc_x), accel_raw_to_ms2((int16_t)acc_y),
             accel_raw_to_ms2((int16_t)acc_z), accel_raw_to_g((int16_t)acc_x),
             accel_raw_to_g((int16_t)acc_y), accel_raw_to_g((int16_t)acc_z),
             gyr_raw_to_rads((int16_t)gyr_x), gyr_raw_to_rads((int16_t)gyr_y),
             gyr_raw_to_rads((int16_t)gyr_z));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error while reading: %s \n", esp_err_to_name(ret));
    }
}

/* External functions */
void es_bmi270_init(size_t configSize, uint8_t *configBuffer, uint8_t sleep_flag)
{
    if (configBuffer == NULL)
    {
        ESP_LOGE(TAG, "Config file is NULL\n");
        return;
    }

    ESP_ERROR_CHECK(bmi_init());
    //ESP_ERROR_CHECK(softreset());
    //ESP_ERROR_CHECK(chip_id());
    if(sleep_flag == 0)
    {
        ESP_ERROR_CHECK(softreset());
        ESP_ERROR_CHECK(chip_id());
        ESP_ERROR_CHECK(initialization(configSize, configBuffer));
        ESP_ERROR_CHECK(set_power_mode(SENSOR_POWER_MODE));
        ESP_ERROR_CHECK(set_acc_odr(SENSOR_ACC_ODR));
        ESP_ERROR_CHECK(set_acc_range(SENSOR_ACC_RANGE));
        ESP_ERROR_CHECK(set_gyr_odr(SENSOR_GYR_ODR));
        ESP_ERROR_CHECK(set_gyr_range(SENSOR_GYR_RANGE));
        internal_status();
    }
}


void es_bmi270_measure(float **readingBuffer)
{
    if (readingBuffer == NULL)
    {
        ESP_LOGE(TAG, "Reading buffer is NULL\n");
        return;
    }

    ESP_LOGI(TAG, "Started measuring\n");
    int i = 0;
    while (1)
    {
        if (is_data_ready())
        {
            esp_err_t ret = ESP_OK;
            uint8_t data_data8[12];
            float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
            
            ret = bmi_read(I2C_BMI_NUM, &REG_DATA_8, (uint8_t *)data_data8, sizeof(data_data8));
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Error while reading: %s \n", esp_err_to_name(ret));
            }
            
            acc_x = accel_raw_to_ms2((int16_t)combine_bytes(data_data8[1], data_data8[0]));
            acc_y = accel_raw_to_ms2((int16_t)combine_bytes(data_data8[3], data_data8[2]));
            acc_z = accel_raw_to_ms2((int16_t)combine_bytes(data_data8[5], data_data8[4]));
            gyr_x = gyr_raw_to_rads((int16_t)combine_bytes(data_data8[7], data_data8[6]));
            gyr_y = gyr_raw_to_rads((int16_t)combine_bytes(data_data8[9], data_data8[8]));
            gyr_z = gyr_raw_to_rads((int16_t)combine_bytes(data_data8[11], data_data8[10]));

            // Save data to buffer
            readingBuffer[i][0] = acc_x;
            readingBuffer[i][1] = acc_y;
            readingBuffer[i][2] = acc_z;
            readingBuffer[i][3] = gyr_x;
            readingBuffer[i][4] = gyr_y;
            readingBuffer[i][5] = gyr_z;

            ESP_LOGI(TAG, "<S_%d>  |  Acc: (%.2f, %.2f, %.2f) m/s²  |  Gy: (%.2f, %.2f, %.2f) rad/s", i, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
            
            // delay 200 ms
            vTaskDelay(500 / portTICK_PERIOD_MS);

            i += 1;
            if (i >= BMI270_SEQUENCE_LENGTH)
            {
                break;
            }
        }
    }
}
