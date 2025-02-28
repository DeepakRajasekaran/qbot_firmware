#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "IOList.h"
#include "Drive.h"

const int driveMode = 0;  // 1 for position control, 2 for velocity control

struct DriveData {
    int dir;
    int pos_cmd;
    int pos_fbk;
    int rpm_cmd;
    int rpm_fbk;
};

DriveData L_drive = {0, 0, 0, 0, 0};
DriveData R_drive = {0, 0, 0, 0, 0};

uint16_t batteryPercent = 0;

struct imuData {
    float x; float y; float z;
    float r; float p; float y;
};


#endif // GLOBALS_H