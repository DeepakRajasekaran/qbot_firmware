#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "com.h"

extern String command; // Define the global variable here
#define CMD_DELIMITER '\n'  // End of command

// Robot physical parameters
#define WHEEL_RADIUS 0.033        // meters (adjust to your wheel)
#define WHEEL_BASE 0.16           // meters (distance between wheels)
#define TICKS_PER_REV 718.0       // encoder ticks per revolution

// Safety parameters
#define HEARTBEAT_TIMEOUT_MS 100  // Stop motors if no command for 100ms
#define MAX_TILT_ANGLE 30.0       // degrees - stop if robot tilts too much

// Operating modes
enum RobotMode {
    MODE_IDLE = 0,
    MODE_VELOCITY = 1,
    MODE_DIRECT = 2,
    MODE_ACTION = 3,
    MODE_DIAGNOSTIC = 4
};

// Action states
enum ActionState {
    ACTION_IDLE = 0,
    ACTION_EXECUTING = 1,
    ACTION_DONE = 2,
    ACTION_ERROR = 3
};

// Global state structure
struct RobotState {
    // Current mode
    RobotMode current_mode;
    ActionState action_state;
    uint8_t current_action_id;
    
    // Odometry data
    double x;           // position x (meters)
    double y;           // position y (meters)
    double theta;       // orientation (radians)
    double linear_vel;  // linear velocity (m/s)
    double angular_vel; // angular velocity (rad/s)
    
    // IMU data
    double accel_x, accel_y, accel_z;     // linear acceleration (m/s²)
    double gyro_x, gyro_y, gyro_z;        // angular rates (rad/s)
    double roll, pitch, yaw;              // orientation (radians)
    
    // Command data
    double cmd_linear_vel;    // commanded linear velocity
    double cmd_angular_vel;   // commanded angular velocity
    double cmd_pwm_left;      // direct PWM command left motor (0-100%)
    double cmd_pwm_right;     // direct PWM command right motor (0-100%)
    
    // Safety
    unsigned long last_command_time;  // timestamp of last received command
    bool heartbeat_ok;                // heartbeat status
    bool tilt_ok;                     // tilt safety status
    
    // Previous encoder counts for odometry
    long prev_left_count;
    long prev_right_count;
    unsigned long prev_odom_time;
};

// Global robot state instance
extern RobotState robot_state;

// Timing variables
extern uint32_t g_last_loop_us;
extern float    g_dt_sec;
extern uint32_t g_last_cmd_time;

// Utility functions
double wrapAngle(double angle);
void initGlobals();
bool isValidCommand(const String &cmd);

#endif // GLOBALS_H