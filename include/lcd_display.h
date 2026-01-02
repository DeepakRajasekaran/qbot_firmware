#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "globals.h"

#define LCD_ADDRESS 0x27  // Common I2C address for LCD
#define LCD_COLS 16
#define LCD_ROWS 2

class LCDDisplay {
public:
    LCDDisplay();
    
    bool init();
    void update();
    void showBootMessage();
    void showError(String message);
    
private:
    void updateEyes();
    void showIdleEyes();
    void showVelocityEyes();
    void showDirectEyes();
    void showActionEyes();
    void showDiagnosticEyes();
    void blinkEyes();
    
    LiquidCrystal_I2C lcd;
    unsigned long last_blink_time;
    bool blink_state;
    unsigned long last_update_time;
    RobotMode last_mode;
    bool initialized;
};

#endif // LCD_DISPLAY_H