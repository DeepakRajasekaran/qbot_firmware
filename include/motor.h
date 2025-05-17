#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define MAX_RPM 100.0
#define ENCODER_PPR 2975.0

class Motor {
private:
    uint8_t encA, encB;
    uint8_t pwmPinFwd, pwmPinRev;
    volatile long encoderCount = 0;
    long lastEncoderCount = 0;
    unsigned long lastSpeedCalcTime = 0;

    double input = 0, output = 0, setpoint = 0;
    double kp, ki, kd;

    PID pid;
    PID_ATune tuner;
    bool tuning;

    int eepromBaseAddr;

    void loadPIDFromEEPROM();
    void savePIDToEEPROM();

public:
    Motor(uint8_t encoderA, uint8_t encoderB,
          uint8_t pwmFwd, uint8_t pwmRev,
          int eepromAddr);

    void init(void (*isrCallback)());
    void handleEncoderISR();
    void updateSpeed();
    void runPID();
    void runMotorAt(double rpm);
    void tune();

    long getEncoderCount() const;
    double getRPM() const;
    double getOutput() const;
    bool isTuning() const;
};

#endif
