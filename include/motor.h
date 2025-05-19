#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define MAX_RPM 100.0
#define ENCODER_PPR 718.0

class Motor {
public:
    Motor(uint8_t fwd_pin, uint8_t rev_pin, uint8_t encA, uint8_t encB);

    volatile double currentRPM = 0.0; // Real-time RPM value

    void attach();
    void runAt(double command_rpm);
    void tuneMotor();
    double getRPM();
    long getCount();

    void enc_ISR();

private:
    uint8_t m_fwd_pin;
    uint8_t m_rev_pin;
    uint8_t m_encA;
    uint8_t m_encB;

    volatile long m_encoder_count;
    unsigned long m_last_millis;

    double m_input = 0, m_output = 0, m_setpoint = 0, m_kp = 0.0, m_ki = 0.0, m_kd = 0.0;
    PID m_pid = PID(&m_input, &m_output, &m_setpoint, m_kp, m_ki, m_kd, DIRECT);

    // Static interrupt handlers
    static void handleInterrupt0();
    static void handleInterrupt1();
};

#endif
