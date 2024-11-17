#include "motor.h"
#include "mpu9250.h"

static const char* TAG = "Motor";

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

void TaskMotor(void* pvParameters) {

    while (1)
    {
        if (!getbIsCalibrated()) {
            vTaskDelay(DELAY_MOTOR / portTICK_PERIOD_MS);
            continue;
        }

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 2048);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(DELAY_MOTOR / portTICK_PERIOD_MS);
    }
}
