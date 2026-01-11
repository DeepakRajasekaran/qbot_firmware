#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

/* --- CONFIGURATION --- */
const float WHEEL_DIAMETER = 0.065; 
const float WHEEL_BASE     = 0.150; 
const int   ENCODER_PPR    = 1920;  
const float PID_SAMPLE_TIME = 50;   

/* --- MODES --- */
enum OpMode { MODE_IDLE = 0, MODE_VELOCITY = 1, MODE_DIRECT = 2, MODE_ACTION = 3, MODE_DIAGNOSTIC = 4 };
OpMode currentMode = MODE_IDLE;

/* --- PIN DEFINITIONS (AS-WIRED) --- */
const int L_PWM = 11, L_DIR1 = 8, L_DIR2 = 9, L_ENCA = 3, L_ENCB = 5;
const int R_PWM = 10, R_DIR1 = 6, R_DIR2 = 7, R_ENCA = 2, R_ENCB = 4;

/* --- GLOBALS --- */
MPU6050 imu;
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile long encLeft = 0, encRight = 0;
double setpointL = 0, inputL = 0, outputL = 0;
double setpointR = 0, inputR = 0, outputR = 0;

// PID Tunings
double Kp = 1.2, Ki = 5.0, Kd = 0.05; 
PID pidL(&inputL, &outputL, &setpointL, Kp, Ki, Kd, DIRECT);
PID pidR(&inputR, &outputR, &setpointR, Kp, Ki, Kd, DIRECT);

// Odometry Pose
float poseX = 0, poseY = 0, poseTheta = 0;
unsigned long lastProcessTime = 0;

/* --- QUADRATURE ENCODER READERS --- */
void isrLeft()  { (digitalRead(L_ENCA) == digitalRead(L_ENCB)) ? encLeft-- : encLeft++; }
void isrRight() { (digitalRead(R_ENCA) == digitalRead(R_ENCB)) ? encRight++ : encRight--; }

/* --- MOTOR CORE --- */
void stopMotors() {
    analogWrite(L_PWM, 0); analogWrite(R_PWM, 0);
    setpointL = 0; setpointR = 0;
    outputL = 0;   outputR = 0;
}

void driveMotor(int pwmPin, int d1, int d2, float val, bool inverse) {
    // val is -255 to 255 (from PID output)
    int pwm = constrain(abs((int)val), 0, 255);
    bool forward = val > 0;
    if (inverse) forward = !forward;

    digitalWrite(d1, forward ? HIGH : LOW);
    digitalWrite(d2, forward ? LOW : HIGH);
    analogWrite(pwmPin, pwm);
}

/* --- UI/LCD --- */
byte eyeOpen[8]  = {0x00,0x0E,0x1F,0x1F,0x1F,0x1F,0x0E,0x00};
byte eyeBlink[8] = {0x00,0x00,0x00,0x1F,0x1F,0x00,0x00,0x00};

void updateLCD() {
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    if (millis() - lastBlink > (blinkState ? 150 : 3000)) {
        blinkState = !blinkState;
        lastBlink = millis();
        lcd.setCursor(5, 0); lcd.write(blinkState ? 1 : 0);
        lcd.setCursor(10, 0); lcd.write(blinkState ? 1 : 0);
    }
    lcd.setCursor(0, 1);
    switch(currentMode) {
        case MODE_IDLE:       lcd.print("MODE: IDLE    "); break;
        case MODE_VELOCITY:   lcd.print("MODE: VELOCITY"); break;
        case MODE_DIRECT:     lcd.print("MODE: DIRECT  "); break;
        case MODE_DIAGNOSTIC: lcd.print("MODE: DIAG    "); break;
        default:              lcd.print("MODE: ACTION  "); break;
    }
}

/* --- NAVIGATION & SENSORS --- */
void processOdometry(float dt) {
    static long lastEncL = 0, lastEncR = 0;
    long dL = encLeft - lastEncL;
    long dR = encRight - lastEncR;
    lastEncL = encLeft; lastEncR = encRight;

    float distL = (dL / (float)ENCODER_PPR) * (PI * WHEEL_DIAMETER);
    float distR = (dR / (float)ENCODER_PPR) * (PI * WHEEL_DIAMETER);

    // Current RPM for PID feedback
    inputL = (distL / dt) * 60.0 / (PI * WHEEL_DIAMETER);
    inputR = (distR / dt) * 60.0 / (PI * WHEEL_DIAMETER);

    float dCenter = (distL + distR) / 2.0;
    float dTheta = (distR - distL) / WHEEL_BASE;

    poseX += dCenter * cos(poseTheta + dTheta/2.0);
    poseY += dCenter * sin(poseTheta + dTheta/2.0);
    poseTheta += dTheta;
}

void sendFeedback() {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Format: f x y theta ax ay az gx gy gz measL measR
    Serial.print("f ");
    Serial.print(poseX); Serial.print(" "); Serial.print(poseY); Serial.print(" "); Serial.print(poseTheta); Serial.print(" ");
    Serial.print(ax); Serial.print(" "); Serial.print(ay); Serial.print(" "); Serial.print(az); Serial.print(" ");
    Serial.print(gx); Serial.print(" "); Serial.print(gy); Serial.print(" "); Serial.print(gz); Serial.print(" ");
    Serial.print(inputL); Serial.print(" "); Serial.println(inputR);
}

/* --- HOST COMMUNICATIONS --- */
void handleSerial() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'm') { 
            currentMode = (OpMode)Serial.parseInt();
            stopMotors(); 
        } 
        else if (cmd == 'v' && currentMode == MODE_VELOCITY) {
            float v = Serial.parseFloat();
            float w = Serial.parseFloat();
            float targetL = v - (w * WHEEL_BASE / 2.0);
            float targetR = v + (w * WHEEL_BASE / 2.0);
            setpointL = (targetL * 60.0) / (PI * WHEEL_DIAMETER);
            setpointR = (targetR * 60.0) / (PI * WHEEL_DIAMETER);
        }
        else if (cmd == 'd' && currentMode == MODE_DIRECT) {
            float pwm_l = Serial.parseFloat();
            float pwm_r = Serial.parseFloat();
            driveMotor(L_PWM, L_DIR1, L_DIR2, pwm_l, false);
            driveMotor(R_PWM, R_DIR1, R_DIR2, pwm_r, true);
        }
        else if (cmd == 's') { 
            currentMode = MODE_IDLE; 
            stopMotors(); 
        }
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(L_PWM, OUTPUT); pinMode(L_DIR1, OUTPUT); pinMode(L_DIR2, OUTPUT);
    pinMode(R_PWM, OUTPUT); pinMode(R_DIR1, OUTPUT); pinMode(R_DIR2, OUTPUT);
    pinMode(L_ENCA, INPUT); pinMode(L_ENCB, INPUT);
    pinMode(R_ENCA, INPUT); pinMode(R_ENCB, INPUT);

    attachInterrupt(digitalPinToInterrupt(L_ENCA), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENCA), isrRight, CHANGE);

    lcd.init(); lcd.backlight();
    lcd.createChar(0, eyeOpen); lcd.createChar(1, eyeBlink);
    imu.initialize();

    pidL.SetOutputLimits(-255, 255);
    pidR.SetOutputLimits(-255, 255);
    pidL.SetMode(AUTOMATIC);
    pidR.SetMode(AUTOMATIC);
}

void loop() {
    handleSerial();
    
    unsigned long now = millis();
    if (now - lastProcessTime >= PID_SAMPLE_TIME) {
        float dt = (now - lastProcessTime) / 1000.0;
        lastProcessTime = now;
        
        processOdometry(dt);
        
        if (currentMode == MODE_VELOCITY) {
            pidL.Compute();
            pidR.Compute();
            driveMotor(L_PWM, L_DIR1, L_DIR2, outputL, false);
            driveMotor(R_PWM, R_DIR1, R_DIR2, outputR, true); // Motor R is INVERSE
        }
        
        sendFeedback();
        updateLCD();
    }
}