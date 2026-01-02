#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "globals.h"
#include "motor.h"

class Odometry {
public:
    Odometry();
    
    void update(Motor& leftMotor, Motor& rightMotor);
    void reset();
    
    // Getters
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getLinearVel() const;
    double getAngularVel() const;
    
private:
    void computeOdometry(long left_count, long right_count, unsigned long current_time);
    void computeVelocities(double delta_s, double delta_theta, double delta_time);
};

#endif // ODOMETRY_H