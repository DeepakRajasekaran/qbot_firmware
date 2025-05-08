#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Preprocessor constants
#define MAX_RPM 300.0
#define ENCODER_PPR 20.0

class Motor {
private:
    uint8_t encA, encB, pwmPin;
    volatile long encoderCount = 0;
    long lastEncoderCount = 0;
    unsigned long lastSpeedCalcTime = 0;

    double input = 0, output = 0, setpoint = 0;
    double kp, ki, kd;

    PID pid;
    PID_ATune tuner;
    bool tuning;

public:
    Motor(uint8_t encoderA, uint8_t encoderB, uint8_t motorPwmPin);

    // Initialization (handles pinMode, attachInterrupt, PID setup)
    void init(void (*isrCallback)());

    // ISR should call this
    void handleEncoderISR();

    // Speed update and PID logic
    void updateSpeed();
    void runPID();

    // Set target RPM and tuning
    void runMotorAt(double rpm);
    void tune();

    // Getters
    long getEncoderCount() const;
    double getRPM() const;
    double getOutput() const;
    bool isTuning() const;
};

#endif
