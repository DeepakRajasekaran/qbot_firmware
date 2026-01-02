#pragma once
#include <stdint.h>

void imu_init();
void imu_update();

extern bool imu_ok;

extern int16_t imu_ax;
extern int16_t imu_ay;
extern int16_t imu_az;

extern int16_t imu_gx;
extern int16_t imu_gy;
extern int16_t imu_gz;