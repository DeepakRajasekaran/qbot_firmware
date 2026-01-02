#include "state_manager.h"
#include "motor.h" // Ensure this is included for motor_set_pwm

StateManager::StateManager() : initialized(false) {
    last_feedback_time = 0;
    feedback_interval = 50; // Send feedback every 50ms (20Hz)
}

void StateManager::init() {
    // Initialize global state
    initGlobals();
    
    // Create module instances
    odometry = new Odometry();
    // imu = new IMU();
    lcd = new LCDDisplay();
    drive = new Drive();
    
    // Initialize modules
    // if (!imu->init()) {
    //     Serial.println("ERROR: IMU initialization failed!");
    // }
    
    if (!lcd->init()) {
        Serial.println("ERROR: LCD initialization failed!");
    }
    
    lcd->showBootMessage();
    
    initialized = true;
    Serial.println("State Manager initialized.");
}

void StateManager::update() {
    if (!initialized) return;
    
    // Update all sensors first
    updateSensors();
    
    // Update safety systems
    updateSafety();
    
    // Update actuators based on current mode
    updateActuators();
    
    // Handle communication
    updateCommunication();
    
    // Update LCD display
    lcd->update();
}

void StateManager::updateSensors() {
    // Update IMU data
    // imu->update();
    
    // Odometry is updated in main loop after motor updates
}

void StateManager::updateActuators() {
    switch (robot_state.current_mode) {
        case MODE_IDLE:
            updateIdleMode();
            break;
        case MODE_VELOCITY:
            updateVelocityMode();
            break;
        case MODE_DIRECT:
            updateDirectMode();
            break;
        case MODE_ACTION:
            updateActionMode();
            break;
        case MODE_DIAGNOSTIC:
            updateDiagnosticMode();
            break;
    }
}

void StateManager::updateCommunication() {
    // Read incoming commands
    readCommand();
    
    // Send feedback at regular intervals
    unsigned long current_time = millis();
    if (current_time - last_feedback_time >= feedback_interval) {
        sendFeedback();
        last_feedback_time = current_time;
    }
}

void StateManager::updateSafety() {
    // Check heartbeat timeout
    if (!robot_state.heartbeat_ok) {
        handleModeTransition(MODE_IDLE);
    }
    
    // Check tilt safety
    if (!robot_state.tilt_ok) {
        handleModeTransition(MODE_IDLE);
        lcd->showError("TILT DETECTED!");
    }
}

void StateManager::handleModeTransition(RobotMode new_mode) {
    RobotMode old_mode = robot_state.current_mode;
    
    // Exit current mode
    switch (old_mode) {
        case MODE_ACTION:
            robot_state.action_state = ACTION_IDLE;
            break;
        default:
            break;
    }
    
    // Set new mode
    robot_state.current_mode = new_mode;
    
    // Enter new mode
    switch (new_mode) {
        case MODE_IDLE:
            robot_state.cmd_linear_vel = 0.0;
            robot_state.cmd_angular_vel = 0.0;
            robot_state.cmd_pwm_left = 0.0;
            robot_state.cmd_pwm_right = 0.0;
            break;
        case MODE_ACTION:
            robot_state.action_state = ACTION_EXECUTING;
            last_action_time = millis();
            break;
        default:
            break;
    }
    
    Serial.print("Mode transition: ");
    Serial.print(old_mode);
    Serial.print(" -> ");
    Serial.println(new_mode);
}

void StateManager::updateIdleMode() {
    // Motors should be stopped
    robot_state.cmd_linear_vel = 0.0;
    robot_state.cmd_angular_vel = 0.0;
}

void StateManager::updateVelocityMode() {
    // Drive module will handle velocity-to-RPM conversion
    // Commands are already set in robot_state by parseCommand()
}

void StateManager::updateDirectMode() {
    // Drive module will handle PWM-to-RPM conversion
    // Commands are already set in robot_state by parseCommand()
}

void StateManager::updateActionMode() {
    // Check action timeout (10 seconds max)
    if (millis() - last_action_time > 10000) {
        robot_state.action_state = ACTION_ERROR;
        handleModeTransition(MODE_IDLE);
    }
    
    // Drive module handles action execution
}

void StateManager::updateDiagnosticMode() {
    // Could add diagnostic routines here
    // For now, just run motors at low speed for testing
}

static bool fault_latched = false;

void state_manager_update() {
    // ...existing fault detection block...
    fault_latched = true;
    // ...existing code...

    if (fault_latched) {
        motor_set_pwm(0);
        return;
    }

    // ...existing state transitions...
}