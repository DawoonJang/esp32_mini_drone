#ifndef __MOTOR_H__

#include "driver/ledc.h"
#include "sys_data.h"

#define MOTOR_GPIO 19
#define LEDC_TIMER LEDC_TIMER_0        // LEDC 타이머
#define LEDC_MODE 0 // 고속 모드 LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0    // LEDC 채널
#define LEDC_FREQUENCY 5000            // PWM 주파수
#define MAX_DUTY 2048
#define DELAY_MOTOR 100

void init_motor(void);
void TaskMotor(void*);

#endif /* motor.h */
