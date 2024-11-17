#ifndef __MPU9250_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sys_data.h"

#define I2C_MASTER_SCL_IO 22      // I2C SCL 핀 번호
#define I2C_MASTER_SDA_IO 21      // I2C SDA 핀 번호
#define I2C_MASTER_NUM I2C_NUM_0  // I2C 포트 번호
#define I2C_MASTER_FREQ_HZ 400000 // I2C 클럭 속도
#define MPU9250_ADDR 0x68         // MPU9250 I2C 주소
#define PWR_MGMT_1 0x6B           // 전원 관리 레지스터
#define ACCEL_XOUT_H 0x3B         // 가속도 데이터 시작 주소
#define INIT_SAMPLE_CNT 1000
#define DELAY_MPU9250 50

typedef struct _st_mpu9250 {
    SINT16 accel_x;
    SINT16 accel_y;
    SINT16 accel_z;
    SINT16 gyro_x;
    SINT16 gyro_y;
    SINT16 gyro_z;
} ST_MPU9250;

void init_i2c_master(void);
void TaskMPU9250(void*);
#endif /* mpu9250.h */
