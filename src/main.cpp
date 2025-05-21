#include "motor.h"

// Create two motor instances
Motor leftMotor(INVERSE, A0, 6, 7, 3, 5, 0); // Motor 1: pwm = a0, in1=6, in2=7, encA=3, encB=5
Motor rightMotor(DIRECT, A1, 8, 9, 2, 4, 24); // Motor 2: pwm = a1, in1=8, in2=9, encA=2, encB=4

void setup() {
    Serial.begin(9600); // Initialize Serial communication
    delay(1000);        // Wait for Serial Monitor to initialize

    // Initialize both motors
    leftMotor.attach();
    rightMotor.attach();

    Serial.println("Motors initialized.");

    leftMotor.tuneMotor();  // Tune left motor
    rightMotor.tuneMotor(); // Tune right motor

    // leftMotor.setTuningParams(2.0, 1.0, 0.5); // kp, ki, kd for left motor
    // rightMotor.setTuningParams(2.0, 1.0, 0.5); // kp, ki, kd for right motor

    delay(1000); // Wait for 1 second before starting
}

void loop() {
    // Define a fixed setpoint for testing
    double command_rpm = 60.0; // Target RPM

    // Run the motors with the setpoint
    leftMotor.runAt(command_rpm, CLOSED_LOOP); // run at ___rpm in ___mode
    rightMotor.runAt(command_rpm, CLOSED_LOOP); 

    Serial.print(">");
    Serial.print("Command RPM: ");
    Serial.print(command_rpm);

    // Print feedback (RPM) to Serial Monitor
    Serial.print(", Motor 1 RPM: ");
    Serial.print(leftMotor.getRpm());
    //Serial.print(" Theta: ");
    //Serial.print(leftMotor.getPos());

    Serial.print(", Motor 2 RPM: ");
    Serial.println(rightMotor.getRpm());
    //Serial.print(" Theta: ");
    //Serial.println(rightMotor.getPos());

    delay(10); // Wait for 1 second before the next iteration
}