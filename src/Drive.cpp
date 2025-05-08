#include "Drive.h"

// Motor instances (adjust pins as needed)
Motor leftMotor(2, 4, 5, 0);       // encA, encB, PWM, EEPROM address 0
Motor rightMotor(3, 7, 6, 30);     // separate EEPROM slot

static DriveMode currentMode = RUN_MODE;

// === ISR Callbacks ===
void leftEncoderISR() {
    leftMotor.handleEncoderISR();
}

void rightEncoderISR() {
    rightMotor.handleEncoderISR();
}

// === Initialization ===
void initDrive() {
    leftMotor.init(leftEncoderISR);
    rightMotor.init(rightEncoderISR);
}

// === Set target speed ===
void setTargetRPM(double leftRPM, double rightRPM) {
    leftMotor.runMotorAt(leftRPM);
    rightMotor.runMotorAt(rightRPM);
}

// === Set system mode ===
void setDriveMode(DriveMode mode) {
    currentMode = mode;

    if (currentMode == MAINTENANCE_MODE) {
        leftMotor.tune();
        rightMotor.tune();
    }
}

// === Periodic update ===
void updateDrive() {
    if (currentMode == RUN_MODE) {
        leftMotor.updateSpeed();
        rightMotor.updateSpeed();

        leftMotor.runPID();
        rightMotor.runPID();
    }
}

// === RPM Accessors ===
double getLeftRPM() {
    return leftMotor.getRPM();
}

double getRightRPM() {
    return rightMotor.getRPM();
}
