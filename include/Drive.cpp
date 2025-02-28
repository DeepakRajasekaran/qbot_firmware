// filepath: /home/username/Documents/PlatformIO/Projects/LowerLevel/include/Drive.cpp
#include "Drive.h"

void readEncoder()
{
    L_ticks += (digitalRead(L_ENCB_PIN) == HIGH) ? 1 : -1;
    R_ticks += (digitalRead(R_ENCB_PIN) == HIGH) ? 1 : -1;
}

void resetEncoder()
{
    L_ticks = 0;
    R_ticks = 0;
}
void setupEncoder()
{
    pinMode(L_ENCA_PIN, INPUT);
    pinMode(L_ENCB_PIN, INPUT);
    pinMode(R_ENCA_PIN, INPUT);
    pinMode(R_ENCB_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(L_ENCA_PIN), readEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(L_ENCB_PIN), readEncoder, RISING);
}


void setupDrive()
{
    pinMode(L_FWD_PIN, OUTPUT);
    pinMode(L_REV_PIN, OUTPUT);
    pinMode(R_FWD_PIN, OUTPUT);
    pinMode(R_REV_PIN, OUTPUT);
}

void driveSetup()
{
    setupEncoder();
    setupDrive();
}

void drive() {
}
