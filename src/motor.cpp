#include "motor.h"

Motor::Motor(uint8_t encoderA, uint8_t encoderB, uint8_t motorPwmPin)
    : encA(encoderA), encB(encoderB), pwmPin(motorPwmPin),
      kp(2.0), ki(5.0), kd(1.0),
      pid(&input, &output, &setpoint, kp, ki, kd, DIRECT),
      tuner(&input, &output), tuning(false)
{}

void Motor::init(void (*isrCallback)()) {
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encA), isrCallback, RISING);

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);

    // AutoTune configuration
    tuner.SetControlType(1);        // PID
    tuner.SetNoiseBand(1);          // Sensitivity
    tuner.SetOutputStep(50);        // Aggressiveness
    tuner.SetLookbackSec(10);       // History window
}

void Motor::handleEncoderISR() {
    if (digitalRead(encB) == LOW)
        encoderCount--;
    else
        encoderCount++;
}

void Motor::updateSpeed() {
    unsigned long now = millis();
    if (now - lastSpeedCalcTime >= 100) {
        long delta = encoderCount - lastEncoderCount;
        input = (delta / ENCODER_PPR) * 60.0;
        lastEncoderCount = encoderCount;
        lastSpeedCalcTime = now;
    }
}

void Motor::runPID() {
    if (!tuning) {
        pid.Compute();
        analogWrite(pwmPin, output);
    }
}

void Motor::runMotorAt(double rpm) {
    setpoint = constrain(rpm, -MAX_RPM, MAX_RPM);
}

void Motor::tune() {
    tuning = true;
    analogWrite(pwmPin, 127);  // Mid-power to start oscillation

    unsigned long start = millis();
    while (!tuner.Runtime()) {
        updateSpeed();  // Feed encoder speed while tuning
        if (millis() - start > 15000) break;  // Timeout
    }

    kp = tuner.GetKp();
    ki = tuner.GetKi();
    kd = tuner.GetKd();

    pid.SetTunings(kp, ki, kd);
    tuning = false;
}

long Motor::getEncoderCount() const {
    return encoderCount;
}

double Motor::getRPM() const {
    return input;
}

double Motor::getOutput() const {
    return output;
}

bool Motor::isTuning() const {
    return tuning;
}
