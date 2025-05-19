#include "motor.h"

// Static array to store Motor instances for interrupt handling
static Motor* motorInstances[2] = {nullptr, nullptr};

Motor::Motor(uint8_t pwm_pin, uint8_t fwd_pin, uint8_t rev_pin, uint8_t encA, uint8_t encB)
    : m_pwm_pin(pwm_pin), m_fwd_pin(fwd_pin), m_rev_pin(rev_pin), m_encA(encA), m_encB(encB), m_encoder_count(0), m_last_micros(0) {}

void Motor::attach() {
    // Set forward and reverse pins as output
    pinMode(m_pwm_pin, OUTPUT);
    pinMode(m_fwd_pin, OUTPUT);
    pinMode(m_rev_pin, OUTPUT);

    // Set encoder pins as input
    pinMode(m_encA, INPUT);
    pinMode(m_encB, INPUT);

    // Initialize PID controller
    m_pid.SetMode(AUTOMATIC);
    m_pid.SetOutputLimits(-255, 255); // Limit output to PWM range

    // Initialize timing
    m_last_micros = micros();

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
    unsigned long current_micros = micros();
    unsigned long time_elapsed = current_micros - m_last_micros;

    if (time_elapsed > 0) { // Avoid division by zero
        // Calculate counts per second using time elapsed
        m_counts_per_sec = 1.0e6 / time_elapsed;

        // Calculate RPM
        m_current_rpm_raw = (m_counts_per_sec / ENCODER_PPR) * 60.0;

        // Calculate position in degrees
        m_pos_theta = (m_encoder_count / ENCODER_PPR) * 360.0;    
    }

    // update last_micros
    m_last_micros = current_micros;
}

void Motor::runAt(double targetRPM) {
    // Map targetRPM to PWM setpoint (-255 to 255)
    m_setpoint = map((int)targetRPM, -100, 100, -255, 255);

    float raw_rpm;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Safe access to shared variable
        raw_rpm = m_current_rpm_raw;
    }

    // Apply low-pass filter (Exponential Moving Average) to the raw RPM
    const float alpha = 0.1;
    m_current_rpm_filtered = (1 - alpha) * m_current_rpm_filtered + alpha * raw_rpm;

    // Map filtered RPM to PID input
    m_input = map((int)m_current_rpm_filtered, -100, 100, -255, 255);

    // Compute the PID output
    if (m_pid.Compute()) {
        if (m_output > 0) {
            digitalWrite(m_fwd_pin, HIGH);
            digitalWrite(m_rev_pin, LOW);
        } else {
            digitalWrite(m_fwd_pin, LOW);
            digitalWrite(m_rev_pin, HIGH);
        }
        analogWrite(m_pwm_pin, abs((int)m_output));
    }
}


void Motor::openLoopRunAt(double targetRPM) {
    // Map targetRPM from -100~100 to -255~255 for PWM control directly in setpoint
    m_setpoint = map((int)targetRPM, -100, 100, -255, 255);
    
    // Set direction using digitalWrite and write absolute output to PWM pin
    if (m_setpoint > 0) {
        digitalWrite(m_fwd_pin, HIGH);
        digitalWrite(m_rev_pin, LOW);
    } else {
        digitalWrite(m_fwd_pin, LOW);
        digitalWrite(m_rev_pin, HIGH);
    }
    analogWrite(m_pwm_pin, abs((int)m_setpoint));
}

double Motor::getRpm() {
    return m_current_rpm_filtered; // Return the real-time RPM value
}

long Motor::getCount() {
    return m_encoder_count;
}

double Motor::getPos(){
    return m_pos_theta; // Return the position in radians
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
