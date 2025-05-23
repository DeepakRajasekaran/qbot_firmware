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
    pinMode(this->m_in1, OUTPUT);
    pinMode(this->m_in2, OUTPUT);

    pinMode(this->m_encA, INPUT);
    pinMode(this->m_encB, INPUT);

    this->m_pid.SetMode(AUTOMATIC);
    this->m_pid.SetOutputLimits(-255, 255);

    this->m_last_micros = micros();

    // Load PID tuning parameters from EEPROM
    // this->loadFromEEPROM(this->m_eeprom_start_address);

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
        this->m_encoder_count += (digitalRead(this->m_encB) ? 1 : -1);
    } else if (this->m_type == INVERSE) {
        this->m_encoder_count += (digitalRead(this->m_encB) ? -1 : 1);
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
    this->m_setpoint = map((int)targetRPM, -100, 100, -255, 255);

    float raw_rpm = 0.0;

    // Check if the elapsed time since the last encoder interrupt exceeds 1 ms
    unsigned long current_micros = micros();
    if ((current_micros - this->m_last_micros) > 1000) { // If no encoder updates for >1 ms
        this->m_current_rpm_raw = 0.0; // Reset raw RPM
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Safe access to shared variable
        raw_rpm = this->m_current_rpm_raw;
    }

    // Apply low-pass filter (Exponential Moving Average)
    const float alpha = 0.1; // Smoothing factor
    this->m_current_rpm_filtered = this->epmFilter(alpha, raw_rpm);

    // Map filtered RPM to PID input
    this->m_input = map((int)this->m_current_rpm_filtered, -100, 100, -255, 255);

    if (mode == CLOSED_LOOP) {
        if (this->m_pid.Compute()) {
            this->setDirection(this->m_output);
            analogWrite(this->m_pwm_pin, abs((int)this->m_output));
        }
    } else if (mode == OPEN_LOOP) {
        this->setDirection(this->m_setpoint);
        analogWrite(this->m_pwm_pin, abs((int)this->m_setpoint));
    }
}

// void Motor::tuneMotor() {
//     double input = 0, output = 0;
//     PID_ATune aTune(&input, &output);

//     // Configure autotune parameters
//     aTune.SetNoiseBand(0.5);
//     aTune.SetOutputStep(50);
//     aTune.SetLookbackSec(10);
//     aTune.SetControlType(1); // PI or PID

//     // Set initial output to zero
//     output = 0;
//     analogWrite(this->m_pwm_pin, 0);

//     unsigned long start = millis();
//     bool tuning = true;

//     // Define the step size for RPM increments
//     const double stepSize = MAX_RPM / 10.0; // Divide the range into 10 steps
//     double targetRPM = 0;

//     while (tuning) {
//         // Increment the target RPM in steps only after 1 minute
//         static unsigned long lastStepChange = start;
//         if (millis() - lastStepChange > 6000) { // 1 minute has passed
//             targetRPM += stepSize;
//             lastStepChange = millis();
//             if (targetRPM > MAX_RPM) {
//             tuning = false; // Stop tuning when the max RPM is reached
//             break;
//             }
//         }

//         Serial.print("Tuning for target RPM: ");
//         Serial.println(targetRPM);

//         // Simulate closed-loop: update input from filtered RPM
//         input = this->getRpm();

//         // Run autotune step
//         int result = aTune.Runtime();
//         if (result != 0) { // Check if tuning is complete
//             tuning = false;
//         }

//         // Apply output to motor
//         this->setDirection(output); // Set motor direction based on output
//         analogWrite(this->m_pwm_pin, abs((int)output)); // Apply PWM signal to motor

//         // Timeout to avoid infinite loop
//         if (millis() - start > 60000) { // 1-minute timeout
//             Serial.println("Autotune timeout!");
//             break;
//         }

//         delay(10); // Small delay to allow processing
//     }

//     // Retrieve tuned parameters
//     double kp = aTune.GetKp();
//     double ki = aTune.GetKi();
//     double kd = aTune.GetKd();

//     // Save and apply new PID parameters
//     if (kp > 0 && ki >= 0 && kd >= 0) { // Ensure valid parameters
//         this->setTuningParams(kp, ki, kd);
//         Serial.println("Autotune complete. Parameters saved:");
//         Serial.print("Kp: "); Serial.println(kp);
//         Serial.print("Ki: "); Serial.println(ki);
//         Serial.print("Kd: "); Serial.println(kd);
//     } else {
//         Serial.println("Autotune failed. Invalid parameters.");
//     }

//     // Stop the motor after tuning
//     analogWrite(this->m_pwm_pin, 0);
// }

// void Motor::loadFromEEPROM(uint8_t address) {
//     EEPROM.get(address, this->m_kp);
//     address += sizeof(this->m_kp);

//     EEPROM.get(address, this->m_ki);
//     address += sizeof(this->m_ki);

//     EEPROM.get(address, this->m_kd);

//     this->m_pid.SetTunings(this->m_kp, this->m_ki, this->m_kd);
// }

// void Motor::saveToEEPROM(uint8_t address) {
//     EEPROM.put(address, this->m_kp);
//     address += sizeof(this->m_kp);

//     EEPROM.put(address, this->m_ki);
//     address += sizeof(this->m_ki);

//     EEPROM.put(address, this->m_kd);
// }

double Motor::epmFilter(double alpha, double input) {
    return (1 - alpha) * this->m_current_rpm_filtered + alpha * input;
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
    this->m_kp = kp;
    this->m_ki = ki;
    this->m_kd = kd;

    this->m_pid.SetTunings(this->m_kp, this->m_ki, this->m_kd);
    // this->saveToEEPROM(this->m_eeprom_start_address);
}

void Motor::setDirection(double output) {
    if (this->m_type == DIRECT) {
        if (output > 0) {               // Forward
            digitalWrite(this->m_in1, HIGH);
            digitalWrite(this->m_in2, LOW);
        } else {
            digitalWrite(this->m_in1, LOW);   // Reverse
            digitalWrite(this->m_in2, HIGH);
        }
    } else if (this->m_type == INVERSE) {
        if (output > 0) {
            digitalWrite(this->m_in1, LOW);   // Forward
            digitalWrite(this->m_in2, HIGH);
        } else {
            digitalWrite(this->m_in1, HIGH);  // Reverse
            digitalWrite(this->m_in2, LOW);
        }
    }
}

double Motor::getRpm() {
    // Use the same alpha as in runAt, e.g., 0.1
    return epmFilter(0.1, this->m_current_rpm_raw);
}

long Motor::getCount() {
    return this->m_encoder_count;
}

double Motor::getPos() {
    return this->m_pos_theta; // Return the position in degrees
}


double Motor::getRpmRaw() {
    return this->m_current_rpm_raw; // Return the raw RPM value
}

int Motor::getOutput() {
    return this->m_output; // Return the last output value
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
