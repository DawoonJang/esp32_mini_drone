//----------------------------------MPU9250 Accelerometer and Gyroscope C library--------------------------------
// Copyright (c) 2019, Alex Mous
// Licensed under the CC BY-NC SA 4.0

//--------------------------------MODIFY THESE PARAMETERS--------------------------------

#define GYRO_RANGE 0 // Select gyroscope range (default: 0)
// Gyroscope Range
// 0 +/- 250 degrees/second
// 1 +/- 500 degrees/second
// 2 +/- 1000 degrees/second
// 3 +/- 2000 degrees/second

#define ACCEL_RANGE 0 // Select accelerometer range (default: 0)
// Accelerometer Range
// 0 +/- 2g
// 1 +/- 4g
// 2 +/- 8g
// 3 +/- 16g

// Offsets (provide your own values from calibration)
#define A_OFF_X 19402
#define A_OFF_Y -2692
#define A_OFF_Z -8625
#define G_OFF_X -733
#define G_OFF_Y 433
#define G_OFF_Z -75

//----------------------------------------------------------------------------------------

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

{
    "configurations": [
        {
            "name": "Win32",
            "includePath": [
                "${workspaceFolder}/**",
                "C:/Users/wnsrk/Documents/GitHub/esp32_mini_drone/main",
                "C:/path/to/other/includes"  // 필요한 다른 경로가 있다면 추가합니다.
            ],
            "defines": [],
            "windowsSdkVersion": "10.0.19041.0",
            "compilerPath": "C:/path/to/compiler",  // 컴파일러 경로를 확인하여 필요시 수정합니다.
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "windows-msvc-x64"
        }
    ],
    "version": 4
}

#define TAU 0.05 // Complementary filter coefficient
#define RAD_TO_DEG 57.29577951308 // Radians to degrees

// Sensitivity values based on selected range
#if GYRO_RANGE == 1
    #define GYRO_SENS 65.5
    #define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
    #define GYRO_SENS 32.8
    #define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
    #define GYRO_SENS 16.4
    #define GYRO_CONFIG 0b00011000
#else
    #define GYRO_SENS 131.0
    #define GYRO_CONFIG 0b00000000
#endif

#if ACCEL_RANGE == 1
    #define ACCEL_SENS 8192.0
    #define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
    #define ACCEL_SENS 4096.0
    #define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
    #define ACCEL_SENS 2048.0
    #define ACCEL_CONFIG 0b00011000
#else
    #define ACCEL_SENS 16384.0
    #define ACCEL_CONFIG 0b00000000
#endif

// Global Variables
int f_dev;
int MPU9250_addr;
float ax, ay, az, gr, gp, gy;
float accel_angle[3], gyro_angle[3], angle[3];
bool first_run = true;
struct timespec start_time, end_time;
float dt;

// Initialize MPU9250
bool init_MPU9250(int addr) {
    MPU9250_addr = addr;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-1");
    f_dev = open(filename, O_RDWR);
    if (f_dev < 0) {
        perror("Failed to open the i2c bus");
        return false;
    }
    if (ioctl(f_dev, I2C_SLAVE, MPU9250_addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        return false;
    }
    return true;
}

// Update accelerometer and gyroscope readings
void update_MPU9250() {
    // Example read operation, actual implementation may vary
    ax = i2c_smbus_read_word_data(f_dev, 0x3B) / ACCEL_SENS - A_OFF_X;
    ay = i2c_smbus_read_word_data(f_dev, 0x3D) / ACCEL_SENS - A_OFF_Y;
    az = i2c_smbus_read_word_data(f_dev, 0x3F) / ACCEL_SENS - A_OFF_Z;

    gr = i2c_smbus_read_word_data(f_dev, 0x43) / GYRO_SENS - G_OFF_X;
    gp = i2c_smbus_read_word_data(f_dev, 0x45) / GYRO_SENS - G_OFF_Y;
    gy = i2c_smbus_read_word_data(f_dev, 0x47) / GYRO_SENS - G_OFF_Z;

    clock_gettime(CLOCK_REALTIME, &end_time);
    dt = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
    start_time = end_time;
}

// Get raw accelerometer data
void get_accel_raw(float *x, float *y, float *z) {
    *x = ax;
    *y = ay;
    *z = az;
}

// Get raw gyroscope data
void get_gyro_raw(float *roll, float *pitch, float *yaw) {
    *roll = gr;
    *pitch = gp;
    *yaw = gy;
}

// Get processed accelerometer data
void get_accel(float *x, float *y, float *z) {
    *x = ax / ACCEL_SENS;
    *y = ay / ACCEL_SENS;
    *z = az / ACCEL_SENS;
}

// Get processed gyroscope data
void get_gyro(float *roll, float *pitch, float *yaw) {
    *roll = gr / GYRO_SENS;
    *pitch = gp / GYRO_SENS;
    *yaw = gy / GYRO_SENS;
}

// Calculate angles using complementary filter
void calculate_angles() {
    if (first_run) {
        accel_angle[0] = atan2(ay, az) * RAD_TO_DEG;
        accel_angle[1] = atan2(-ax, az) * RAD_TO_DEG;
        gyro_angle[0] = accel_angle[0];
        gyro_angle[1] = accel_angle[1];
        first_run = false;
    } else {
        gyro_angle[0] += gr * dt;
        gyro_angle[1] += gp * dt;

        accel_angle[0] = atan2(ay, az) * RAD_TO_DEG;
        accel_angle[1] = atan2(-ax, az) * RAD_TO_DEG;

        angle[0] = TAU * accel_angle[0] + (1 - TAU) * gyro_angle[0];
        angle[1] = TAU * accel_angle[1] + (1 - TAU) * gyro_angle[1];
    }
}

// Get calculated angle for a specific axis
float get_angle(int axis) {
    return angle[axis];
}
