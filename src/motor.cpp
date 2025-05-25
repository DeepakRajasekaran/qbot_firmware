#include "motor.h"
// #include <EEPROM.h> // Include EEPROM library

// Static array to store Motor instances for interrupt handling
static Motor* motorInstances[2] = {nullptr, nullptr};

Motor::Motor(
    uint8_t type,
    uint8_t pwm_pin,
    uint8_t in1,
    uint8_t in2,
    uint8_t encA,
    uint8_t encB,
    uint8_t eeprom_address
)
    : m_type(type),
      m_pwm_pin(pwm_pin),
      m_in1(in1),
      m_in2(in2),
      m_encA(encA),
      m_encB(encB),
      m_eeprom_start_address(eeprom_address),
      m_encoder_count(0),
      m_last_micros(0)
{}

void Motor::attach() {
    pinMode(this->m_pwm_pin, OUTPUT);
    analogWrite(this->m_pwm_pin, 0); // initial idle state

    pinMode(this->m_in1, OUTPUT);
    pinMode(this->m_in2, OUTPUT);

    pinMode(this->m_encA, INPUT);
    pinMode(this->m_encB, INPUT);

    this->m_pid.SetMode(AUTOMATIC);
    this->m_pid.SetOutputLimits(0, 255);

    this->m_last_micros = micros();

    // Register this instance in the static array
    if (motorInstances[0] == nullptr) {
        motorInstances[0] = this;
        attachInterrupt(digitalPinToInterrupt(this->m_encA), Motor::handleInterrupt0, RISING);
    } else if (motorInstances[1] == nullptr) {
        motorInstances[1] = this;
        attachInterrupt(digitalPinToInterrupt(this->m_encA), Motor::handleInterrupt1, RISING);
    }
}

void Motor::enc_ISR() {
    if (this->m_type == DIRECT) {
        this->m_encoder_count += (digitalRead(this->m_encB) ? -1 : 1);
    } else if (this->m_type == INVERSE) {
        this->m_encoder_count += (digitalRead(this->m_encB) ? 1 : -1);
    }

    unsigned long current_micros = micros();
    unsigned long time_elapsed = current_micros - this->m_last_micros;

    if (time_elapsed > 0) { // Avoid division by zero
        // Calculate counts per second using time elapsed
        this->m_counts_per_sec = 1.0e6 / time_elapsed;

        // Calculate RPM
        this->m_current_rpm_raw = (this->m_counts_per_sec / ENCODER_PPR) * 60.0;

        // Determine motor spinning direction
        this->m_current_rpm_raw = (this->m_encoder_count < this->m_last_encoder_count) ? -this->m_current_rpm_raw : this->m_current_rpm_raw;

        // Calculate position in degrees
        this->m_pos_theta = (this->m_encoder_count / ENCODER_PPR) * 360.0;
    }

    this->m_last_micros = current_micros;
    this->m_last_encoder_count = this->m_encoder_count;
}

void Motor::runAt(double targetRPM, uint8_t mode) {
    
    double raw_rpm = 0.0;
    
    // Only reset RPM if targetRPM is 0 and no encoder updates for >1 ms
    unsigned long timeElapsed = micros() - this->m_last_micros;

    if (targetRPM == 0 && timeElapsed > RPM_TIMEOUT_uS) {
        this->m_current_rpm_raw = 0.0;
    }

    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Safe access to shared variable
        raw_rpm = this->m_current_rpm_raw;
    }
    
    // Apply filter
    this->m_current_rpm_filtered = this->epmFilter(raw_rpm);
    
    // Map filtered RPM to PID input
    this->m_input = map(abs((int)this->m_current_rpm_filtered), 0, MAX_RPM, 0, 255);
    // this->m_input = map(abs((int)this->m_current_rpm_raw), 0, MAX_RPM, 0, 255);
    this->m_setpoint = map(abs((int)targetRPM), 0, MAX_RPM, 0, 255);

    if (mode == CLOSED_LOOP) {
        if (this->m_pid.Compute()) {
            this->setDirection(targetRPM);
            analogWrite(this->m_pwm_pin, this->m_output);
        }
    } else if (mode == OPEN_LOOP) {
        this->setDirection(targetRPM);
        analogWrite(this->m_pwm_pin, this->m_setpoint);
    }
}

double Motor::epmFilter(double input) {
    // Exponential Moving Average filter
    double alpha = 0.069; // Filter coefficient (tune as needed)
    return (1 - alpha) * this->m_current_rpm_filtered + alpha * input;
}

void Motor::setTuningParams(double kp, double ki, double kd) {
    this->m_kp = kp;
    this->m_ki = ki;
    this->m_kd = kd;

    this->m_pid.SetTunings(this->m_kp, this->m_ki, this->m_kd);
}

void Motor::setDirection(double command) {
    if (this->m_type == DIRECT) {
        if (command > 0) {               // Forward
            digitalWrite(this->m_in1, HIGH);
            digitalWrite(this->m_in2, LOW);
        } else {
            digitalWrite(this->m_in1, LOW);   // Reverse
            digitalWrite(this->m_in2, HIGH);
        }
    } else if (this->m_type == INVERSE) {
        if (command > 0) {
            digitalWrite(this->m_in1, LOW);   // Forward
            digitalWrite(this->m_in2, HIGH);
        } else {
            digitalWrite(this->m_in1, HIGH);  // Reverse
            digitalWrite(this->m_in2, LOW);
        }
    }
}

long Motor::getCount() { return this->m_encoder_count; }

int Motor::getOutput() { return this->m_output; }             // Return the last output value}

double Motor::getRpm() { return epmFilter(this->m_current_rpm_raw ); }

double Motor::getPos() { return this->m_pos_theta; }          // Return the position in degrees}

double Motor::getRpmRaw() { return this->m_current_rpm_raw; } // Return the raw RPM value}

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
