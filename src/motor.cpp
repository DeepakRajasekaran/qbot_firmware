#include "motor.h"

// Static array to store Motor instances for interrupt handling
static Motor* motorInstances[2] = {nullptr, nullptr};

Motor::Motor(uint8_t type, uint8_t pwm_pin, uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB)
    :  m_type(type), m_pwm_pin(pwm_pin), m_in1(in1), m_in2(in2), m_encA(encA), m_encB(encB), m_encoder_count(0), m_last_micros(0) {}

void Motor::attach() {
    
    pinMode(m_pwm_pin, OUTPUT);
    pinMode(m_in1, OUTPUT);
    pinMode(m_in2, OUTPUT);

    pinMode(m_encA, INPUT);
    pinMode(m_encB, INPUT);

    m_pid.SetMode(AUTOMATIC);
    m_pid.SetOutputLimits(-255, 255); 

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

    if (m_type == DIRECT) {
        m_encoder_count += (digitalRead(m_encB) ? 1 : -1);
    } else if (m_type == INVERSE) {
        m_encoder_count += (digitalRead(m_encB) ? -1 : 1);
    }

    // m_encoder_count += (digitalRead(m_encB) ? 1 : -1);

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

    m_last_micros = current_micros;
}

void Motor::runAt(double targetRPM, uint8_t mode) {

    m_setpoint = map((int)targetRPM, -100, 100, -255, 255);

    float raw_rpm;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Safe access to shared variable
        raw_rpm = m_current_rpm_raw;
    }

    // low-pass filter (Exponential Moving Average)
    const float alpha = 0.3;
    const int windowSize = 5;
    m_current_rpm_filtered = movingAverageFilter(m_current_rpm_raw, windowSize);

    m_input = map((int)m_current_rpm_filtered, -100, 100, -255, 255);

    if (mode == CLOSED_LOOP) {

        if (m_pid.Compute()) {
            setDirection(m_output);
            analogWrite(m_pwm_pin, abs((int)m_output));
        }
    } else if (mode == OPEN_LOOP) {

        setDirection(m_setpoint);
        analogWrite(m_pwm_pin, abs((int)m_setpoint));
    }
}

double Motor::epmFilter(double alpha, double input) {
    return (1 - alpha) * m_current_rpm_filtered + alpha * input;
}

double Motor::movingAverageFilter(double input, int windowSize) {
    static double* window = nullptr;
    static int currentWindowSize = 0;
    static int index = 0;
    static int count = 0;

    if (window == nullptr || currentWindowSize != windowSize) {
        if (window != nullptr) {
            delete[] window;
        }
        window = new double[windowSize];
        for (int i = 0; i < windowSize; ++i) window[i] = 0.0;
        currentWindowSize = windowSize;
        index = 0;
        count = 0;
    }

    window[index] = input;
    index = (index + 1) % windowSize;
    if (count < windowSize) count++;

    double sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += window[i];
    }

    return sum / count;
}

void Motor::setTuningParams(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    m_pid.SetTunings(m_kp, m_ki, m_kd);
}

void Motor::setDirection(double output){
    if (m_type == DIRECT) {
        if (output > 0) {               // Forward
            digitalWrite(m_in1, HIGH);
            digitalWrite(m_in2, LOW);
        } else {
            digitalWrite(m_in1, LOW);   // Reverse
            digitalWrite(m_in2, HIGH);
        }
    } else if (m_type == INVERSE) {
        if (output > 0) {
            digitalWrite(m_in1, LOW);   // Forward
            digitalWrite(m_in2, HIGH);
        } else {
            digitalWrite(m_in1, HIGH);  // Reverse
            digitalWrite(m_in2, LOW);
        }
    }
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
