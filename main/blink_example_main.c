#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 22      // I2C SCL 핀 번호
#define I2C_MASTER_SDA_IO 21      // I2C SDA 핀 번호
#define I2C_MASTER_NUM I2C_NUM_0  // I2C 포트 번호
#define I2C_MASTER_FREQ_HZ 400000 // I2C 클럭 속도
#define MPU9250_ADDR 0x68         // MPU9250 I2C 주소
#define PWR_MGMT_1 0x6B           // 전원 관리 레지스터
#define ACCEL_XOUT_H 0x3B         // 가속도 데이터 시작 주소

static const char *TAG = "MPU9250";

// I2C 초기화 함수
static void i2c_master_init(void) {
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

// I2C 레지스터에 값 쓰기
static esp_err_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
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

// I2C 레지스터에서 다중 바이트 읽기
static esp_err_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
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

void app_main(void) {
    uint8_t raw_data[14];
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    int16_t offset_ax = 0, offset_ay = 0, offset_az = 0;
    int16_t offset_gx = 0, offset_gy = 0, offset_gz = 0;
    const int sample_count = 2000; // 샘플 개수
    esp_err_t ret;

    // I2C 초기화
    i2c_master_init();

    // MPU9250 초기화
    ret = i2c_write_byte(MPU9250_ADDR, PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 초기화 실패");
        return;
    }
    ESP_LOGI(TAG, "MPU9250 초기화 성공");

    // 초기 데이터 수집 및 오프셋 계산
    ESP_LOGI(TAG, "초기 데이터 수집 시작...");
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < sample_count; i++) {
        ret = i2c_read_bytes(MPU9250_ADDR, ACCEL_XOUT_H, raw_data, 14);
        if (ret == ESP_OK) {
            accel_x = (raw_data[0] << 8) | raw_data[1];
            accel_y = (raw_data[2] << 8) | raw_data[3];
            accel_z = (raw_data[4] << 8) | raw_data[5];
            gyro_x = (raw_data[8] << 8) | raw_data[9];
            gyro_y = (raw_data[10] << 8) | raw_data[11];
            gyro_z = (raw_data[12] << 8) | raw_data[13];

            sum_ax += accel_x;
            sum_ay += accel_y;
            sum_az += accel_z;
            sum_gx += gyro_x;
            sum_gy += gyro_y;
            sum_gz += gyro_z;
        } else {
            ESP_LOGE(TAG, "센서 데이터 읽기 실패");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    offset_ax = sum_ax / sample_count;
    offset_ay = sum_ay / sample_count;
    offset_az = sum_az / sample_count;
    offset_gx = sum_gx / sample_count;
    offset_gy = sum_gy / sample_count;
    offset_gz = sum_gz / sample_count;

    ESP_LOGI(TAG, "초기 데이터 수집 완료!");
    ESP_LOGI(TAG, "Accel 오프셋 (X,Y,Z): %6d %6d %6d", offset_ax, offset_ay, offset_az);
    ESP_LOGI(TAG, "Gyro 오프셋 (X,Y,Z): %6d %6d %6d", offset_gx, offset_gy, offset_gz);

    // 센서 데이터 읽기 및 보정
    while (1) {
        ret = i2c_read_bytes(MPU9250_ADDR, ACCEL_XOUT_H, raw_data, 14);
        if (ret == ESP_OK) {
            accel_x = ((raw_data[0] << 8) | raw_data[1]) - offset_ax;
            accel_y = ((raw_data[2] << 8) | raw_data[3]) - offset_ay;
            accel_z = ((raw_data[4] << 8) | raw_data[5]) - offset_az;
            gyro_x = ((raw_data[8] << 8) | raw_data[9]) - offset_gx;
            gyro_y = ((raw_data[10] << 8) | raw_data[11]) - offset_gy;
            gyro_z = ((raw_data[12] << 8) | raw_data[13]) - offset_gz;

            ESP_LOGI(TAG, "Accel (X,Y,Z): %6d %6d %6d | Gyro (X,Y,Z): %6d %6d %6d",
                     accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        } else {
            ESP_LOGE(TAG, "센서 데이터 읽기 실패");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

