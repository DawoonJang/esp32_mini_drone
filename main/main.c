#include "mpu9250.h"
#include "motor.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "driver/ledc.h"
// #include "esp_log.h"
// #include "sdkconfig.h"

void app_main(void) {
    init_i2c_master();
    init_motor();

    BaseType_t task1 = xTaskCreate(TaskMPU9250, "MPU9250", 2048, NULL, 5, NULL);
    BaseType_t task2 = xTaskCreate(TaskMotor, "MOTOR", 2048, NULL, 5, NULL);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to prevent hogging the CPU
    }
}
