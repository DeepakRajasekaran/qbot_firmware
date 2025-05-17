#include "Drive.h"

Motor leftMotor(2, 4, 5, 6, 0);    // encA, encB, fwdPWM, revPWM, EEPROM addr
Motor rightMotor(3, 7, 9, 10, 30); // encA, encB, fwdPWM, revPWM, EEPROM addr

DriveMode currentMode = RUN_MODE;

// ISR callbacks
void leftEncoderISR()  { leftMotor.handleEncoderISR(); } // interruptServiceRoutine - ISR
void rightEncoderISR() { rightMotor.handleEncoderISR(); }

void initDrive() {
    leftMotor.init(leftEncoderISR);
    rightMotor.init(rightEncoderISR);
}

void setDriveMode(DriveMode mode) {
    currentMode = mode;

    if (mode == MAINTENANCE_MODE) {
        Serial.println("Tuning Left Motor...");
        leftMotor.tune();

        Serial.println("Tuning Right Motor...");
        rightMotor.tune();
    }
}

void setTargetRPM(double leftRPM, double rightRPM) {
    leftMotor.runMotorAt(leftRPM);
    rightMotor.runMotorAt(rightRPM);
}

void updateDrive() {
    if (currentMode == RUN_MODE) {
        leftMotor.updateSpeed();
        leftMotor.runPID();

        rightMotor.updateSpeed();
        rightMotor.runPID();

        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {
            lastPrint = millis();
            Serial.print("Left RPM: ");
            Serial.print(leftMotor.getRPM());
            Serial.print("\tPWM: ");
            Serial.print(leftMotor.getOutput());

            Serial.print(" | Right RPM: ");
            Serial.print(rightMotor.getRPM());
            Serial.print("\tPWM: ");
            Serial.println(rightMotor.getOutput());
        }
    }
}

double getLeftRPM() {
    return leftMotor.getRPM();
}

double getRightRPM() {
    return rightMotor.getRPM();
}
