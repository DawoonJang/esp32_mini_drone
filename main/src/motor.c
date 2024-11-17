#include "motor.h"

void init_motor(void) {
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

// ESP_LOGI(TAG, "모터 속도를 증가시킵니다.");
// for (int duty = 0; duty <= MAX_DUTY; duty += 500) {
//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

//     // MPU9250 가속도, 자이로 데이터 읽기
//     read_mpu9250_data();

//     vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
// }

// ESP_LOGI(TAG, "모터 속도를 감소시킵니다.");
// for (int duty = MAX_DUTY; duty >= 0; duty -= 500) {
//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

//     // MPU9250 가속도, 자이로 데이터 읽기
//     read_mpu9250_data();

//     vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기