#ifndef __MOTOR_H__

#include "driver/ledc.h"

#define MOTOR_GPIO 19
#define LEDC_TIMER LEDC_TIMER_0        // LEDC 타이머
#define LEDC_MODE LEDC_HIGH_SPEED_MODE // 고속 모드
#define LEDC_CHANNEL LEDC_CHANNEL_0    // LEDC 채널
#define LEDC_FREQUENCY 5000            // PWM 주파수
#define MAX_DUTY 8191                  // 최대 Duty 값

void init_motor(void);

#endif /* motor.h */
