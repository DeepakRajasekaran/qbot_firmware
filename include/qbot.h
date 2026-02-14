#ifndef QBOT_H
#define QBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

/* --- MACROS --- */
#define OFF 0
#define TEST 1
#define COM 2

/* --- CONFIGURATION --- */
const float WHEEL_DIAMETER = 0.065; 
const float WHEEL_BASE     = 0.150;  
const int   ENCODER_PPR    = 1920;  
const float PID_SAMPLE_TIME = 50;   
const float MAX_RPM        = 110.0;

/* --- MODES --- */
enum OpMode { MODE_IDLE = 0, MODE_VELOCITY = 1, MODE_DIRECT = 2, MODE_ACTION = 3, MODE_DIAGNOSTIC = 4 };

/* --- PIN DEFINITIONS --- */
const int L_PWM = 11, L_DIR1 = 8, L_DIR2 = 9, L_ENCA = 3, L_ENCB = 5;
const int R_PWM = 10, R_DIR1 = 6, R_DIR2 = 7, R_ENCA = 2, R_ENCB = 4;

/* --- GLOBAL EXTERNS --- */
extern OpMode currentMode;
extern LiquidCrystal_I2C lcd;
extern volatile long encLeft, encRight;
extern double setpointL, inputL, outputL;
extern double setpointR, inputR, outputR;
extern PID pidL;
extern PID pidR;
extern int telemetryMode;
extern float poseX, poseY, poseTheta;
extern unsigned long lastProcessTime;

/* --- FUNCTION PROTOTYPES --- */
void isrLeft();
void isrRight();
void stopMotors();
void driveMotor(int pwmPin, int d1, int d2, float val, bool inverse);
void updateLCD();
void processOdometry(float dt);
void sendFeedback();
void handleSerial();

#endif