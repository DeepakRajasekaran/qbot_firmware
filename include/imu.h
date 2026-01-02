#pragma once

void imu_init();
void imu_update();

extern bool imu_ok;

extern float imu_ax;
extern float imu_ay;
extern float imu_az;

extern float imu_gx;
extern float imu_gy;
extern float imu_gz;

extern float imu_roll;
extern float imu_pitch;
extern float imu_yaw;