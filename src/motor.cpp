#include "motor.h"

Motor::Motor(uint8_t encoderA, uint8_t encoderB, uint8_t motorPwmPin, int eepromAddr)
    : encA(encoderA), encB(encoderB), pwmPin(motorPwmPin),
      kp(2.0), ki(5.0), kd(1.0),
      pid(&input, &output, &setpoint, kp, ki, kd, DIRECT),
      tuner(&input, &output), tuning(false),
      eepromBaseAddr(eepromAddr)
{}

void Motor::init(void (*isrCallback)()) {
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encA), isrCallback, RISING);

    loadPIDFromEEPROM();  // Load saved values

    pid.SetTunings(kp, ki, kd);
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);

    tuner.SetControlType(1);
    tuner.SetNoiseBand(1);
    tuner.SetOutputStep(50);
    tuner.SetLookbackSec(10);
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
    analogWrite(pwmPin, 127);

    unsigned long start = millis();
    while (!tuner.Runtime()) {
        updateSpeed();
        if (millis() - start > 15000) break;
    }

    kp = tuner.GetKp();
    ki = tuner.GetKi();
    kd = tuner.GetKd();

    pid.SetTunings(kp, ki, kd);
    savePIDToEEPROM();  // Save new values
    tuning = false;
}

void Motor::savePIDToEEPROM() {
    EEPROM.put(eepromBaseAddr, kp);
    EEPROM.put(eepromBaseAddr + sizeof(double), ki);
    EEPROM.put(eepromBaseAddr + 2 * sizeof(double), kd);
}

void Motor::loadPIDFromEEPROM() {
    EEPROM.get(eepromBaseAddr, kp);
    EEPROM.get(eepromBaseAddr + sizeof(double), ki);
    EEPROM.get(eepromBaseAddr + 2 * sizeof(double), kd);

    // Optional: validate values
    if (isnan(kp) || isnan(ki) || isnan(kd) || kp <= 0 || ki < 0 || kd < 0) {
        kp = 2.0; ki = 5.0; kd = 1.0;  // Defaults
    }
}

long Motor::getEncoderCount() const { return encoderCount; }
double Motor::getRPM() const { return input; }
double Motor::getOutput() const { return output; }
bool Motor::isTuning() const { return tuning; }
