#ifndef IOLIST_H
#define IOLIST_H

// Motor 1 (Left Motor) - DIRECT type
#define M1_PWM 11
#define M1_IN1 8
#define M1_IN2 9
#define M1_ENC_A 3  // Interrupt pin
#define M1_ENC_B 5

// Motor 2 (Right Motor) - INVERSE type  
#define M2_PWM 10
#define M2_IN1 6
#define M2_IN2 7
#define M2_ENC_A 2  // Interrupt pin
#define M2_ENC_B 4

// IMU (I2C)
#define SDA_PIN A4  // Default I2C SDA
#define SCL_PIN A5  // Default I2C SCL

// LCD (I2C) - shares same I2C bus as IMU
#define LCD_I2C_ADDR 0x27

// Battery monitoring (optional)
#define BATTERY_PIN A0

// Status LED (optional)
#define STATUS_LED 13

#endif // IOLIST_H