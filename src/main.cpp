#include "motor.h"
#include "globals.h"
#include "state_manager.h"
#include "odometry.h"
#include "drive.h"
#include "imu.h"

#define CONTROL_PERIOD_US 10000 // 10 ms control period (100 Hz)

// Create motor instances - FIXED PIN ASSIGNMENTS
Motor leftMotor(DIRECT, 11, 8, 9, 3, 5, 24);   // PWM=11, IN1=8, IN2=9, encA=3, encB=5
Motor rightMotor(INVERSE, 10, 6, 7, 2, 4, 0);  // PWM=10, IN1=6, IN2=7, encA=2, encB=4

// Create module instances
StateManager stateManager;
Odometry odometry;
Drive driveControl;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== QBOT ROBOT INITIALIZING ===");
    
    // Initialize communication
    initCom(115200);
    
    // Initialize global state
    initGlobals();
    
    // Initialize motors
    leftMotor.attach();
    rightMotor.attach();
    
    // Set PID tuning parameters
    leftMotor.setTuningParams(1.3, 3.2, 0.1);
    rightMotor.setTuningParams(1.3, 3.5, 0.1);
    
    // Initialize drive control
    driveControl.init(leftMotor, rightMotor);
    
    // Initialize state manager (this will init IMU, LCD, etc.)
    stateManager.init();
    
    // Initialize IMU
    imu_init();
    
    Serial.println("=== INITIALIZATION COMPLETE ===");
    Serial.println("Commands: VEL v w | DRV pwmL pwmR | ACT id | MODE id | STOP | RESET");
    
    delay(500);
}

void loop() {
    uint32_t now = micros();
    if ((uint32_t)(now - g_last_loop_us) < CONTROL_PERIOD_US) {
        return;
    }

    g_dt_sec = (now - g_last_loop_us) * 1e-6f;
    g_last_loop_us = now;
    
    // 1. Update odometry from encoder counts
    odometry.update(leftMotor, rightMotor);
    imu_update();
    // 2. Update state manager (handles IMU, communication, safety, LCD)
    stateManager.update();
    
    // 3. Update drive control based on current mode and commands
    driveControl.update();
}