#include "globals.h"

// Global robot state instance
RobotState robot_state;

String command = "";

void initGlobals() {
    // Initialize robot state
    robot_state.current_mode = MODE_IDLE;
    robot_state.action_state = ACTION_IDLE;
    robot_state.current_action_id = 0;
    
    // Initialize odometry
    robot_state.x = 0.0;
    robot_state.y = 0.0;
    robot_state.theta = 0.0;
    robot_state.linear_vel = 0.0;
    robot_state.angular_vel = 0.0;
    
    // Initialize IMU data
    robot_state.accel_x = robot_state.accel_y = robot_state.accel_z = 0.0;
    robot_state.gyro_x = robot_state.gyro_y = robot_state.gyro_z = 0.0;
    robot_state.roll = robot_state.pitch = robot_state.yaw = 0.0;
    
    // Initialize commands
    robot_state.cmd_linear_vel = 0.0;
    robot_state.cmd_angular_vel = 0.0;
    robot_state.cmd_pwm_left = 0.0;
    robot_state.cmd_pwm_right = 0.0;
    
    // Initialize safety
    robot_state.last_command_time = millis();
    robot_state.heartbeat_ok = true;
    // robot_state.tilt_ok = true;
    
    // Initialize odometry tracking
    robot_state.prev_left_count = 0;
    robot_state.prev_right_count = 0;
    robot_state.prev_odom_time = micros();
}

double wrapAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

uint32_t g_last_loop_us = 0;
float    g_dt_sec       = 0.0f;
uint32_t g_last_cmd_time = 0;