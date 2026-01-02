#include "imu.h"
#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_ADDR 0x69   // AD0 = HIGH

MPU6050 mpu(MPU6050_ADDR);

bool imu_ok = false;

int16_t imu_ax = 0;
int16_t imu_ay = 0;
int16_t imu_az = 0;

int16_t imu_gx = 0;
int16_t imu_gy = 0;
int16_t imu_gz = 0;

void imu_init()
{
    Wire.begin();
    mpu.initialize();
    imu_ok = mpu.testConnection();
}

void imu_update()
{
    if (!imu_ok) return;

    imu_ax = mpu.getAccelerationX();
    imu_ay = mpu.getAccelerationY();
    imu_az = mpu.getAccelerationZ();

    imu_gx = mpu.getRotationX();
    imu_gy = mpu.getRotationY();
    imu_gz = mpu.getRotationZ();
}