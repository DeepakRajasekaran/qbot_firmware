#include "motor.h"

// Create two motor instances
Motor leftMotor(A0, 6, 7, 2, 4); // Motor 1: pwm = a0, fwd=6, rev=7, encA=2, encB=4
Motor rightMotor(A1, 8, 9, 3, 5); // Motor 2: pwm = a1, fwd=8, rev=9, encA=3, encB=5

void setup() {
    Serial.begin(9600); // Initialize Serial communication
    delay(1000);        // Wait for Serial Monitor to initialize

    // Initialize both motors
    leftMotor.attach();
    rightMotor.attach();

    Serial.println("Motors initialized.");
}

void loop() {
    // Define a fixed setpoint for testing
    double command_rpm = 50.0; // Target RPM

    // Run the motors with the setpoint
    leftMotor.runAt(command_rpm, OPEN_LOOP);
    rightMotor.runAt(command_rpm, OPEN_LOOP);

    // Print feedback (RPM) to Serial Monitor
    Serial.print("Motor 1 RPM: ");
    Serial.print(leftMotor.getRpm());
    Serial.print(" Theta: ");
    Serial.print(leftMotor.getPos());

    Serial.print(" | Motor 2 RPM: ");
    Serial.print(rightMotor.getRpm());
    Serial.print(" Theta: ");
    Serial.println(rightMotor.getPos());

    delay(10); // Wait for 1 second before the next iteration
}