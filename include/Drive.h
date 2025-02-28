#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "Globals.h"

const int ticksPerRev = 2925; // per rev at output shaft
const int MAX_RPM     = 100;

int R_ENCA_PIN        = M2_ENC_1;
int R_ENCB_PIN        = M2_ENC_2;
int R_FWD_PIN         = M2_FWD;
int R_REV_PIN         = M2_REV;
long R_ticks          = 0;

int L_ENCA_PIN        = M1_ENC_1;
int L_ENCB_PIN        = M1_ENC_2;
int L_FWD_PIN         = M1_FWD;
int L_REV_PIN         = M1_REV;
long L_ticks          = 0;

void readEncoder();
void resetEncoder();
void setupEncoder();
void setupDrive();

void driveSetup();
// void drive(int left, int right);

#endif // DRIVE_H