#include "qbot.h"

/* --- GLOBALS --- */
OpMode currentMode = MODE_IDLE;
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile long encLeft = 0, encRight = 0;
double setpointL = 0, inputL = 0, outputL = 0;
double setpointR = 0, inputR = 0, outputR = 0;

PID pidL(&inputL, &outputL, &setpointL, 1.3, 3.2, 0.1, DIRECT);
PID pidR(&inputR, &outputR, &setpointR, 1.3, 3.5, 0.1, DIRECT);

// Odometry Pose
int telemetryMode = COM; // 0=OFF, 1=TEST, 2=COM
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
    int pwm = constrain(abs((int)val), 0, 255);
    bool forward = val > 0;
    if (inverse) forward = !forward;

    digitalWrite(d1, forward ? HIGH : LOW);
    digitalWrite(d2, forward ? LOW : HIGH);
    analogWrite(pwmPin, pwm);
}

/* --- UI/LCD --- */
byte eyeOpenTL[8] = {0x00, 0x00, 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F};
byte eyeOpenTR[8] = {0x00, 0x00, 0x18, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F};
byte eyeOpenBL[8] = {0x1F, 0x1F, 0x1F, 0x0F, 0x07, 0x03, 0x00, 0x00};
byte eyeOpenBR[8] = {0x1F, 0x1F, 0x1F, 0x1E, 0x1C, 0x18, 0x00, 0x00};
byte eyeBlinkTL[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F};
byte eyeBlinkTR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F};
byte eyeBlinkBL[8] = {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte eyeBlinkBR[8] = {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void updateLCD() {
    static OpMode lastMode = (OpMode)-1;
    static unsigned long modeChangeTime = 0;
    static bool inTransition = false;
    static bool forceRedraw = true;

    // 1. Handle Mode Change
    if (currentMode != lastMode) {
        lastMode = currentMode;
        inTransition = true;
        modeChangeTime = millis();
        lcd.clear();
        
        // Show Log Message
        lcd.setCursor(0, 0);
        lcd.print("MODE CHANGED:");
        lcd.setCursor(0, 1);
        
        String logMsg = "";
        switch(currentMode) {
            case MODE_IDLE:       logMsg = "IDLE"; break;
            case MODE_VELOCITY:   logMsg = "VELOCITY"; break;
            case MODE_DIRECT:     logMsg = "DIRECT"; break;
            case MODE_DIAGNOSTIC: logMsg = "DIAGNOSTIC"; break;
            default:              logMsg = "ACTION"; break;
        }
        
        // Center the text
        int pad = (16 - logMsg.length()) / 2;
        if (pad < 0) pad = 0;
        for(int i=0; i<pad; i++) lcd.print(" ");
        lcd.print(logMsg);
        return;
    }

    // 2. Handle Transition Timeout
    if (inTransition) {
        if (millis() - modeChangeTime > 3000) {
            inTransition = false;
            lcd.clear();
            forceRedraw = true;
        } else {
            return; // Wait for 3 seconds to pass
        }
    }

    // 3. Draw Eyes (Normal Operation)
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    
    if (millis() - lastBlink > (blinkState ? 150 : 3000)) {
        blinkState = !blinkState;
        lastBlink = millis();
        forceRedraw = true;
    }
    
    if (forceRedraw) {
        int tl = blinkState ? 4 : 0;
        int tr = blinkState ? 5 : 1;
        int bl = blinkState ? 6 : 2;
        int br = blinkState ? 7 : 3;

        // Left Eye (Cols 2,3)
        lcd.setCursor(2, 0); lcd.write(tl);
        lcd.setCursor(3, 0); lcd.write(tr);
        lcd.setCursor(2, 1); lcd.write(bl);
        lcd.setCursor(3, 1); lcd.write(br);
        
        // Right Eye (Cols 12,13)
        lcd.setCursor(12, 0); lcd.write(tl);
        lcd.setCursor(13, 0); lcd.write(tr);
        lcd.setCursor(12, 1); lcd.write(bl);
        lcd.setCursor(13, 1); lcd.write(br);
        
        forceRedraw = false;
    }
}

/* --- NAVIGATION & SENSORS --- */
void processOdometry(float dt) {
    static long lastEncL = 0, lastEncR = 0;
    
    long currEncL, currEncR;
    noInterrupts(); // Disable interrupts to ensure atomic read of 4-byte variables
    currEncL = encLeft;
    currEncR = encRight;
    interrupts();   // Re-enable interrupts immediately

    long dL = currEncL - lastEncL;
    long dR = currEncR - lastEncR;
    lastEncL = currEncL; lastEncR = currEncR;

    float distL = (dL / (float)ENCODER_PPR) * (PI * WHEEL_DIAMETER);
    float distR = (dR / (float)ENCODER_PPR) * (PI * WHEEL_DIAMETER);

    inputL = (distL / dt) * 60.0 / (PI * WHEEL_DIAMETER);
    inputR = (distR / dt) * 60.0 / (PI * WHEEL_DIAMETER);

    float dCenter = (distL + distR) / 2.0;
    float dTheta = (distR - distL) / WHEEL_BASE;

    poseX += dCenter * cos(poseTheta + dTheta/2.0);
    poseY += dCenter * sin(poseTheta + dTheta/2.0);
    poseTheta += dTheta;
}

void sendFeedback() {
    
    if (telemetryMode == 2) {
        // COM Mode
        Serial.print("f;");
        Serial.print(poseX); Serial.print(";"); 
        Serial.print(poseY); Serial.print(";"); 
        Serial.print(poseTheta); Serial.print(";");
        Serial.print(inputL); Serial.print(";"); 
        Serial.println(inputR);
    } else if (telemetryMode == 1) {
        // TEST Mode
        Serial.println("---");
        Serial.print("Pose: [x: "); Serial.print(poseX); 
        Serial.print(", y: "); Serial.print(poseY); 
        Serial.print(", th: "); Serial.print(poseTheta); Serial.println("]");
        Serial.print("RPM:  [L: "); Serial.print(inputL); 
        Serial.print(", R: "); Serial.print(inputR); Serial.println("]");
    }
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
            
            // Clamp setpoints to physical limits
            setpointL = constrain(setpointL, -MAX_RPM, MAX_RPM);
            setpointR = constrain(setpointR, -MAX_RPM, MAX_RPM);
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
        else if (cmd == 'r') {
            poseX = 0;
            poseY = 0;
            poseTheta = 0;
        }
        else if (cmd == 't') {
            telemetryMode = Serial.parseInt();
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

    // Prevent loop blocking if serial data is incomplete
    Serial.setTimeout(10);

    lcd.init(); lcd.backlight();
    lcd.createChar(0, eyeOpenTL); lcd.createChar(1, eyeOpenTR);
    lcd.createChar(2, eyeOpenBL); lcd.createChar(3, eyeOpenBR);
    lcd.createChar(4, eyeBlinkTL); lcd.createChar(5, eyeBlinkTR);
    lcd.createChar(6, eyeBlinkBL); lcd.createChar(7, eyeBlinkBR);

    pidL.SetOutputLimits(-255, 255);
    pidR.SetOutputLimits(-255, 255);
    pidL.SetMode(AUTOMATIC);
    pidR.SetMode(AUTOMATIC);
    pidL.SetSampleTime(PID_SAMPLE_TIME);
    pidR.SetSampleTime(PID_SAMPLE_TIME);
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
            driveMotor(R_PWM, R_DIR1, R_DIR2, outputR, true);
        }
        
        if (telemetryMode > 0) sendFeedback();
        updateLCD();
    }
}