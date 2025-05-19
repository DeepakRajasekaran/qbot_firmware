#include "motor.h"

// Static array to store Motor instances for interrupt handling
static Motor* motorInstances[2] = {nullptr, nullptr};

Motor::Motor(uint8_t fwd_pin, uint8_t rev_pin, uint8_t encA, uint8_t encB)
    : m_fwd_pin(fwd_pin), m_rev_pin(rev_pin), m_encA(encA), m_encB(encB), m_encoder_count(0), m_last_millis(0) {}

void Motor::attach() {
    // Set forward and reverse pins as output
    pinMode(m_fwd_pin, OUTPUT);
    pinMode(m_rev_pin, OUTPUT);

    // Set encoder pins as input
    pinMode(m_encA, INPUT);
    pinMode(m_encB, INPUT);

    // Initialize PID controller
    m_pid.SetMode(AUTOMATIC);
    m_pid.SetOutputLimits(-255, 255); // Limit output to PWM range

    // Initialize timing
    m_last_millis = millis();

    // Register this instance in the static array
    if (motorInstances[0] == nullptr) {
        motorInstances[0] = this;
        attachInterrupt(digitalPinToInterrupt(m_encA), Motor::handleInterrupt0, RISING);
    } else if (motorInstances[1] == nullptr) {
        motorInstances[1] = this;
        attachInterrupt(digitalPinToInterrupt(m_encA), Motor::handleInterrupt1, RISING);
    }
}

void Motor::enc_ISR() {
    // Increment or decrement encoder count based on encB state
    m_encoder_count += (digitalRead(m_encB) ? 1 : -1);

    // Calculate current RPM
    unsigned long current_millis = millis();
    unsigned long time_diff = current_millis - m_last_millis;

    if (time_diff > 0) { // Avoid division by zero
        currentRPM = (m_encoder_count / ENCODER_PPR) * (60000.0 / time_diff);
    }

    // Reset timing and encoder count
    m_last_millis = current_millis;
    m_encoder_count = 0;
}

void Motor::runAt(double targetRPM) {
    
    // Map targetRPM and currentRPM from -100~100 to -255~255 for PWM control directly in setpoint and input
    m_setpoint = map((int)targetRPM, -100, 100, -255, 255);
    m_input = map((int)currentRPM, -100, 100, -255, 255);
    
    // Compute the PID output
    if (m_pid.Compute()) {
        // Use the PID output to control the motor
        if (m_output > 0) {
            analogWrite(m_fwd_pin, m_output);
            analogWrite(m_rev_pin, 0);
        } else {
            analogWrite(m_fwd_pin, 0);
            analogWrite(m_rev_pin, -m_output);
        }
    }
    
}

double Motor::getRPM() {
    return currentRPM; // Return the real-time RPM value
}

long Motor::getCount() {
    return m_encoder_count;
}

// Static interrupt handlers
void Motor::handleInterrupt0() {
    if (motorInstances[0]) {
        motorInstances[0]->enc_ISR();
    }
}

void Motor::handleInterrupt1() {
    if (motorInstances[1]) {
        motorInstances[1]->enc_ISR();
    }
}
