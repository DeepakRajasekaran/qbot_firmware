#include "Drive.h"

void setup() {
    Serial.begin(9600);
    delay(1000);

    initDrive();                      // Initialize motors
    setDriveMode(MAINTENANCE_MODE);  // Tune once
    setDriveMode(RUN_MODE);          // Enter run mode

    setTargetRPM(100, 100);          // Set target RPM for both motors
}

void loop() {
    updateDrive();                   // Manage loop logic (PID + printing)
}
