#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <util/atomic.h>
#include <EEPROM.h>

#define MAX_RPM 80.0
#define ENCODER_PPR 718.0

#define DIRECT 0
#define INVERSE 1

#define OPEN_LOOP 2
#define CLOSED_LOOP 3

class Motor {
public:
    Motor(uint8_t type, uint8_t pwm_pin, uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t eeprom_address);

    void attach();
    void runAt(double targetRpm, uint8_t mode);
    // void tuneMotor();
    void setTuningParams(double kp, double ki, double kd);
    double epmFilter(double alpha, double input);
    double movingAverageFilter(double input, int windowSize);

    // Getters
    double getRpm();
    long getCount();
    double getPos();

private:
    // Speed control
    void enc_ISR();
    void setDirection(double output);
    // void loadFromEEPROM(uint8_t address);
    // void saveToEEPROM(uint8_t address);

    uint8_t m_type;
    uint8_t m_pwm_pin;

    // Direction control
    uint8_t m_in1;
    uint8_t m_in2;

    // Feedback
    uint8_t m_encA;
    uint8_t m_encB;

    uint8_t m_eeprom_start_address;

    volatile long m_counts_per_sec = 0;
    volatile long m_encoder_count = 0; // Encoder count
    volatile long m_last_encoder_count = 0; // Last encoder count
    unsigned long m_last_micros;

    volatile double m_current_rpm_raw = 0.0;      // Real-time RPM value
    volatile double m_current_rpm_filtered = 0.0; // Filtered RPM value

    double m_pos_theta = 0.0;

    double m_input = 0, m_output = 0, m_setpoint = 0, m_kp = 1.0, m_ki = 0.0, m_kd = 0.0;
    PID m_pid = PID(&m_input, &m_output, &m_setpoint, m_kp, m_ki, m_kd, DIRECT);

    // Static interrupt handlers
    static void handleInterrupt0();
    static void handleInterrupt1();
};

#endif
