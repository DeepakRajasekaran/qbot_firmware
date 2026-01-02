#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>
#include "globals.h"
#include "motor.h"
#include "odometry.h"
#include "lcd_display.h"
#include "drive.h"
#include "com.h"

class StateManager {
public:
    StateManager();
    
    void init();
    void update();
    void handleModeTransition(RobotMode new_mode);
    
private:
    bool initialized;
    unsigned long last_feedback_time;
    unsigned long feedback_interval;
    unsigned long last_action_time;
    
    // Module instances
    Odometry* odometry;
    // IMU* imu;
    LCDDisplay* lcd;
    Drive* drive;
    
    void updateSensors();
    void updateActuators();
    void updateCommunication();
    void updateSafety();
    
    // Mode-specific updates
    void updateIdleMode();
    void updateVelocityMode();
    void updateDirectMode();
    void updateActionMode();
    void updateDiagnosticMode();
};

#endif // STATE_MANAGER_H