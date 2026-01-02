#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"
#include "globals.h"

class Drive {
public:
    Drive();
    
    void init(Motor& leftMotor, Motor& rightMotor);
    void update();
    void stop();
    
    // Convert velocity commands to wheel RPM
    void velocityToWheelRPM(double linear_vel, double angular_vel, double& left_rpm, double& right_rpm);
    
    // Convert PWM percentage to motor commands
    void pwmToMotorCommands(double pwm_left_percent, double pwm_right_percent, double& left_rpm, double& right_rpm);
    
    // Safety checks
    bool isSafeToMove();
    
private:
    Motor* m_left_motor;
    Motor* m_right_motor;
    
    bool initialized;
    unsigned long last_action_time;
};

#endif // DRIVE_H