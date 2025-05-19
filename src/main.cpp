#include "motor.h"

// Create two motor instances
Motor leftMotor(A0, A1, 2, 4); // Motor 1: fwd=A0, rev=A1, encA=2, encB=4
Motor rightMotor(A2, A3, 3, 5); // Motor 2: fwd=A2, rev=A3, encA=3, encB=5

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
    double command_rpm = 100.0; // Target RPM

    // Run the motors with the setpoint
    leftMotor.runAt(command_rpm);
    rightMotor.runAt(command_rpm);

    // Print feedback (RPM) to Serial Monitor
    Serial.print("Motor 1 RPM: ");
    Serial.print(leftMotor.getRPM());

    Serial.print(" | Motor 2 RPM: ");
    Serial.println(rightMotor.getRPM());

    delay(1000); // Wait for 1 second before the next iteration
}