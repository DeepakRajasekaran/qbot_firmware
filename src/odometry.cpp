#include "odometry.h"

Odometry::Odometry() {
    reset();
}

void Odometry::update(Motor& leftMotor, Motor& rightMotor) {
    unsigned long current_time = micros();
    long left_count = leftMotor.getCount();
    long right_count = rightMotor.getCount();
    
    computeOdometry(left_count, right_count, current_time);
}

void Odometry::reset() {
    robot_state.x = 0.0;
    robot_state.y = 0.0;
    robot_state.theta = 0.0;
    robot_state.linear_vel = 0.0;
    robot_state.angular_vel = 0.0;
    robot_state.prev_left_count = 0;
    robot_state.prev_right_count = 0;
    robot_state.prev_odom_time = micros();
}

void Odometry::computeOdometry(long left_count, long right_count, unsigned long current_time) {
    // Calculate time difference
    double delta_time = (current_time - robot_state.prev_odom_time) / 1.0e6; // convert to seconds
    
    if (delta_time <= 0) return; // Avoid division by zero
    
    // Calculate encoder tick differences
    long delta_left = left_count - robot_state.prev_left_count;
    long delta_right = right_count - robot_state.prev_right_count;
    
    // Convert ticks to wheel angles (radians)
    double delta_theta_left = 2.0 * PI * delta_left / TICKS_PER_REV;
    double delta_theta_right = 2.0 * PI * delta_right / TICKS_PER_REV;
    
    // Calculate wheel arc lengths
    double delta_s_left = WHEEL_RADIUS * delta_theta_left;
    double delta_s_right = WHEEL_RADIUS * delta_theta_right;
    
    // Calculate robot motion
    double delta_s = (delta_s_right + delta_s_left) / 2.0;      // linear displacement
    double delta_theta = (delta_s_right - delta_s_left) / WHEEL_BASE; // angular displacement
    
    // Update robot pose using dead reckoning
    double theta_mid = robot_state.theta + delta_theta / 2.0;
    robot_state.x += delta_s * cos(theta_mid);
    robot_state.y += delta_s * sin(theta_mid);
    robot_state.theta = wrapAngle(robot_state.theta + delta_theta);
    
    // Compute velocities
    computeVelocities(delta_s, delta_theta, delta_time);
    
    // Update previous values
    robot_state.prev_left_count = left_count;
    robot_state.prev_right_count = right_count;
    robot_state.prev_odom_time = current_time;
}

void Odometry::computeVelocities(double delta_s, double delta_theta, double delta_time) {
    robot_state.linear_vel = delta_s / delta_time;
    robot_state.angular_vel = delta_theta / delta_time;
}

// Getters
double Odometry::getX() const { return robot_state.x; }
double Odometry::getY() const { return robot_state.y; }
double Odometry::getTheta() const { return robot_state.theta; }
double Odometry::getLinearVel() const { return robot_state.linear_vel; }
double Odometry::getAngularVel() const { return robot_state.angular_vel; }

void odometry_update() {
    if (g_dt_sec <= 0.0f || g_dt_sec > 0.1f) {
        return;
    }

    // ...existing odometry calculations...
}