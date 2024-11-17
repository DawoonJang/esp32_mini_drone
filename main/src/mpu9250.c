#include "mpu9250.h"

static const char* TAG = "MPU9250";

ST_MPU9250 stMPU9250;
bool bIsCalibrated;

static esp_err_t _i2c_write_byte(uint8_t, uint8_t, uint8_t);
static esp_err_t _i2c_read_bytes(uint8_t, uint8_t, uint8_t*, size_t);

bool getbIsCalibrated(void) {
    return bIsCalibrated;
}

void init_i2c_master(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t _i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t _i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void TaskMPU9250(void* pvParameters) {
    static UINT16 sample_count = 0;

    static SINT16 offset_ax, offset_ay, offset_az,
        offset_gx, offset_gy, offset_gz;

    UINT8 raw_data[14];

    SINT16 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    esp_err_t ret;

    while (1) {
        ret = _i2c_write_byte(MPU9250_ADDR, PWR_MGMT_1, 0x00);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize MPU9250");
            vTaskDelete(NULL);
            continue;;
        }
        ret = _i2c_read_bytes(MPU9250_ADDR, ACCEL_XOUT_H, raw_data, 14);

        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to read sensor data");
            continue;
        }

        accel_x = (SINT16)COMBINE_TWO_BYTES(raw_data[0], raw_data[1]);
        accel_y = (SINT16)COMBINE_TWO_BYTES(raw_data[2], raw_data[3]);
        accel_z = (SINT16)COMBINE_TWO_BYTES(raw_data[4], raw_data[5]);

        gyro_x = (SINT16)COMBINE_TWO_BYTES(raw_data[8], raw_data[9]);
        gyro_y = (SINT16)COMBINE_TWO_BYTES(raw_data[10], raw_data[11]);
        gyro_z = (SINT16)COMBINE_TWO_BYTES(raw_data[12], raw_data[13]);

        if (sample_count < INIT_SAMPLE_CNT) {
            offset_ax = (sample_count * offset_ax + accel_x) / (sample_count + 1);
            offset_ay = (sample_count * offset_ay + accel_y) / (sample_count + 1);
            offset_az = (sample_count * offset_az + accel_z) / (sample_count + 1);

            offset_gx = (sample_count * offset_gx + gyro_x) / (sample_count + 1);
            offset_gy = (sample_count * offset_gy + gyro_y) / (sample_count + 1);
            offset_gz = (sample_count * offset_gz + gyro_z) / (sample_count + 1);

            sample_count++;
            vTaskDelay(DELAY_MPU9250 / portTICK_PERIOD_MS);
            continue;
        }
        else {
            if (!bIsCalibrated) {
                ESP_LOGI(TAG, "Initial data collection completed!");
                bIsCalibrated = true;
            }

            stMPU9250.accel_x = accel_x - offset_ax;
            stMPU9250.accel_y = accel_y - offset_ay;
            stMPU9250.accel_z = accel_z - offset_az;

            stMPU9250.gyro_x = gyro_x - offset_gx;
            stMPU9250.gyro_y = gyro_y - offset_gy;
            stMPU9250.gyro_z = gyro_z - offset_gz;
        }

        ESP_LOGI(TAG, "Accel (X,Y,Z): %6d %6d %6d, Gyro (X,Y,Z): %6d %6d %6d",
            stMPU9250.accel_x, stMPU9250.accel_y, stMPU9250.accel_z,
            stMPU9250.gyro_x, stMPU9250.gyro_y, stMPU9250.gyro_z);

        vTaskDelay(DELAY_MPU9250 / portTICK_PERIOD_MS);
    }
}