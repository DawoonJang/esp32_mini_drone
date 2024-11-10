#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "motor_sensor_example";

#define MOTOR_GPIO 19                  // 모터가 연결된 GPIO 핀
#define LEDC_TIMER LEDC_TIMER_0        // LEDC 타이머
#define LEDC_MODE LEDC_HIGH_SPEED_MODE // 고속 모드
#define LEDC_CHANNEL LEDC_CHANNEL_0    // LEDC 채널
#define LEDC_FREQUENCY 5000            // PWM 주파수
#define MAX_DUTY 8191                  // 최대 Duty 값

// I2C 설정
#define I2C_MASTER_SCL_IO 22           // I2C SCL 핀
#define I2C_MASTER_SDA_IO 21           // I2C SDA 핀
#define I2C_MASTER_NUM I2C_NUM_0       // I2C 포트 번호
#define I2C_MASTER_FREQ_HZ 100000      // I2C 주파수
#define MPU9250_ADDRESS 0x68           // MPU9250 I2C 주소
#define WHO_AM_I_REG 0x75              // 장치 ID 확인 레지스터

// 가속도, 자이로, 자력계 데이터 레지스터 주소
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MAG_XOUT_L 0x03                // AK8963 레지스터 주소 (자력계)

#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B

// I2C 초기화 함수
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void mpu9250_init(void) {
    // PWR_MGMT_1 레지스터 초기화 (디바이스 활성화)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true); // 잠금 해제 (내부 클럭 사용)
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);

    // 가속도 설정 (±2g 설정)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ACCEL_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true); // ±2g
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);

    // 자이로 설정 (±250°/s 설정)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, GYRO_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true); // ±250°/s
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "MPU9250 초기화 완료");
}


// 레지스터에서 16비트 데이터 읽기 함수
static int16_t mpu9250_read_16bit(uint8_t high_addr) {
    uint8_t high_byte, low_byte;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, high_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &high_byte, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &low_byte, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);

    return (int16_t)((high_byte << 8) | low_byte);
}

// 가속도, 자이로, 자력계 데이터 읽기 함수
static void read_mpu9250_data(void) {
    int16_t accel_x = mpu9250_read_16bit(ACCEL_XOUT_H);
    int16_t accel_y = mpu9250_read_16bit(ACCEL_XOUT_H + 2);
    int16_t accel_z = mpu9250_read_16bit(ACCEL_XOUT_H + 4);

    int16_t gyro_x = mpu9250_read_16bit(GYRO_XOUT_H);
    int16_t gyro_y = mpu9250_read_16bit(GYRO_XOUT_H + 2);
    int16_t gyro_z = mpu9250_read_16bit(GYRO_XOUT_H + 4);

    ESP_LOGI(TAG, "가속도 (X, Y, Z): %d, %d, %d", accel_x, accel_y, accel_z);
    ESP_LOGI(TAG, "자이로 (X, Y, Z): %d, %d, %d", gyro_x, gyro_y, gyro_z);
}

// 모터 설정 함수
static void configure_motor(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void app_main(void) {
    // I2C 초기화
    i2c_master_init();
    mpu9250_init();
    // MPU9250 WHO_AM_I 레지스터 확인
    uint8_t who_am_i;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, WHO_AM_I_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &who_am_i, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);

    if (who_am_i == 0x71) {
        ESP_LOGI(TAG, "MPU9250 연결 성공!");
    } else {
        ESP_LOGE(TAG, "MPU9250 연결 실패. WHO_AM_I: 0x%02X", who_am_i);
    }

    // 모터 설정
    configure_motor();

    while (1) {
        ESP_LOGI(TAG, "모터 속도를 증가시킵니다.");
        for (int duty = 0; duty <= MAX_DUTY; duty += 500) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            // MPU9250 가속도, 자이로 데이터 읽기
            read_mpu9250_data();

            vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
        }

        ESP_LOGI(TAG, "모터 속도를 감소시킵니다.");
        for (int duty = MAX_DUTY; duty >= 0; duty -= 500) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            // MPU9250 가속도, 자이로 데이터 읽기
            read_mpu9250_data();

            vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
        }
    }
}
