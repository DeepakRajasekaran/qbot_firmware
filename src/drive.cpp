#include "drive.h"

Drive::Drive() : initialized(false) {
    last_action_time = 0;
}

void Drive::init(Motor& leftMotor, Motor& rightMotor) {
    m_left_motor = &leftMotor;
    m_right_motor = &rightMotor;
    initialized = true;
}

void Drive::update() {
    if (!initialized) return;
    
    // Safety check first
    if (!isSafeToMove()) {
        stop();
        return;
    }
    
    double left_rpm = 0.0, right_rpm = 0.0;
    
    switch (robot_state.current_mode) {
        case MODE_IDLE:
            // Do nothing, motors stop
            left_rpm = right_rpm = 0.0;
            break;
            
        case MODE_VELOCITY:
            // Convert velocity commands to wheel RPM
            velocityToWheelRPM(robot_state.cmd_linear_vel, robot_state.cmd_angular_vel, left_rpm, right_rpm);
            break;
            
        case MODE_DIRECT:
            // Convert PWM percentages to RPM
            pwmToMotorCommands(robot_state.cmd_pwm_left, robot_state.cmd_pwm_right, left_rpm, right_rpm);
            break;
            
        case MODE_DIAGNOSTIC:
            // Diagnostic mode - could run motor tests
            left_rpm = right_rpm = 10.0; // Slow test speed
            break;
        
        case MODE_ACTION:
            // call action handler if needed
            break;
    }
    
    // Apply motor commands
    m_left_motor->runAt(left_rpm, CLOSED_LOOP);
    m_right_motor->runAt(right_rpm, CLOSED_LOOP);
}

void Drive::velocityToWheelRPM(double linear_vel, double angular_vel, double& left_rpm, double& right_rpm) {
    // Differential drive kinematics
    // v_left = linear_vel - (angular_vel * wheel_base) / 2
    // v_right = linear_vel + (angular_vel * wheel_base) / 2
    
    double left_wheel_vel = linear_vel - (angular_vel * WHEEL_BASE) / 2.0;
    double right_wheel_vel = linear_vel + (angular_vel * WHEEL_BASE) / 2.0;
    
    // Convert linear wheel velocity to RPM
    // RPM = (linear_velocity / wheel_circumference) * 60
    double wheel_circumference = 2.0 * PI * WHEEL_RADIUS;
    
    left_rpm = (left_wheel_vel / wheel_circumference) * 60.0;
    right_rpm = (right_wheel_vel / wheel_circumference) * 60.0;
    
    // Clamp to maximum RPM
    left_rpm = constrain(left_rpm, -MAX_RPM, MAX_RPM);
    right_rpm = constrain(right_rpm, -MAX_RPM, MAX_RPM);
}

void Drive::pwmToMotorCommands(double pwm_left_percent, double pwm_right_percent, double& left_rpm, double& right_rpm) {
    // Convert PWM percentage to RPM (simple linear mapping)
    // Clamp PWM to ±100%
    pwm_left_percent = constrain(pwm_left_percent, -100.0, 100.0);
    pwm_right_percent = constrain(pwm_right_percent, -100.0, 100.0);
    
    left_rpm = (pwm_left_percent / 100.0) * MAX_RPM;
    right_rpm = (pwm_right_percent / 100.0) * MAX_RPM;
}

bool Drive::isSafeToMove() {
    return robot_state.heartbeat_ok ;//&& robot_state.tilt_ok;
}

void Drive::stop() {
    if (!initialized) return;
    
    m_left_motor->runAt(0.0, CLOSED_LOOP);
    m_right_motor->runAt(0.0, CLOSED_LOOP);
}