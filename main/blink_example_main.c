/* Motor Control Example using LEDC */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "motor_example";

#define MOTOR_GPIO 19                 // 드론 날개 모터가 연결된 GPIO 핀
#define LEDC_TIMER LEDC_TIMER_0       // LEDC 타이머 0 사용
#define LEDC_MODE LEDC_HIGH_SPEED_MODE // 고속 모드 사용
#define LEDC_CHANNEL LEDC_CHANNEL_0   // LEDC 채널 0 사용
#define LEDC_FREQUENCY 5000           // 주파수: 5 kHz (모터에 적합한 주파수 설정)
#define MAX_DUTY 8191                 // 최대 Duty 값 (13-bit 해상도에서)

static void configure_motor(void)
{
    // LEDC 타이머 설정
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,           // LEDC 속도 모드
        .timer_num = LEDC_TIMER,           // 타이머 번호
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit Duty 해상도
        .freq_hz = LEDC_FREQUENCY,         // PWM 주파수
        .clk_cfg = LEDC_AUTO_CLK           // 자동 클럭 선택
    };
    ledc_timer_config(&ledc_timer);

    // LEDC 채널 설정
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,    // LEDC 인터럽트 비활성화
        .gpio_num = MOTOR_GPIO,            // 모터가 연결된 GPIO 핀
        .duty = 0,                         // 초기 Duty 값
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void app_main(void)
{
    // 모터 제어 설정
    configure_motor();

    while (1) {
        ESP_LOGI(TAG, "모터 속도를 증가시킵니다.");
        // Duty 값을 증가시켜 모터 속도를 높임
        for (int duty = 0; duty <= MAX_DUTY; duty += 500) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
        }

        ESP_LOGI(TAG, "모터 속도를 감소시킵니다.");
        // Duty 값을 감소시켜 모터 속도를 줄임
        for (int duty = MAX_DUTY; duty >= 0; duty -= 500) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
        }
    }
}