#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"

// System modes
enum DriveMode {
    RUN_MODE,
    MAINTENANCE_MODE
};

// Initialize motors and attach interrupts
void initDrive();

// Set system mode
void setDriveMode(DriveMode mode);

// Run one loop iteration of drive logic
void updateDrive();

// Set target RPM for both motors
void setTargetRPM(double leftRPM, double rightRPM);

// Get current RPMs (optional utilities)
double getLeftRPM();
double getRightRPM();

#endif
