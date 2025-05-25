#include "motor.h"

#define period 8 // 125 hz | cycle time (8ms)

// Create two motor instances
Motor leftMotor(DIRECT, 11, 8, 9, 3, 5, 24); // Motor 1: pwm = a0, in1=6, in2=7, encA=3, encB=5
Motor rightMotor(INVERSE, 10, 6, 7, 2, 4, 0); // Motor 2: pwm = a1, in1=8, in2=9, encA=2, encB=4

void setup() {
    Serial.begin(115200); // Initialize Serial communication
    delay(1000);        // Wait for Serial Monitor to initialize

    // Initialize both motors
    leftMotor.attach();
    rightMotor.attach();

    Serial.println("Motors initialized.");

    // leftMotor.tuneMotor();  // Tune left motor
    // rightMotor.tuneMotor(); // Tune right motor

    leftMotor.setTuningParams(1.3, 3.2, 0.1); // kp, ki, kd for left motor
    rightMotor.setTuningParams(1.3, 3.5, 0.1);   

    delay(1000); // Wait for 1 second before starting

}

void loop() {

    int startMillis = millis();



    // Calculate elapsed time and delay to maintain the desired frequency

    int endMillis = millis();
    int elapsedTime = endMillis - startMillis;
    int delayTime = period - elapsedTime;

    if (delayTime > 0) {
        delay(delayTime);
    }
}