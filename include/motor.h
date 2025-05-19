#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <util/atomic.h>

#define MAX_RPM 100.0
#define ENCODER_PPR 718.0

class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t fwd_pin, uint8_t rev_pin, uint8_t encA, uint8_t encB);

    void attach();
    void runAt(double targetRpm);
    void openLoopRunAt(double targetRpm);
    void tuneMotor();

    // Getters
    double getRpm();
    long getCount();
    double getPos();

    
private:
    //speed control
    void enc_ISR();

    uint8_t m_pwm_pin;

    //direction control
    uint8_t m_fwd_pin;
    uint8_t m_rev_pin;

    //feedback
    uint8_t m_encA;
    uint8_t m_encB;

    volatile long m_counts_per_sec = 0;
    volatile long m_encoder_count = 0; // Encoder count
    unsigned long m_last_micros;

    volatile double m_current_rpm_raw = 0.0;      // Real-time RPM value
    volatile double m_current_rpm_filtered = 0.0; // Filtered RPM value

    double m_pos_theta = 0.0;

    double m_input = 0, m_output = 0, m_setpoint = 0, m_kp = 0.0, m_ki = 0.0, m_kd = 0.0;
    PID m_pid = PID(&m_input, &m_output, &m_setpoint, m_kp, m_ki, m_kd, DIRECT);

    // Static interrupt handlers
    static void handleInterrupt0();
    static void handleInterrupt1();
};

#endif
