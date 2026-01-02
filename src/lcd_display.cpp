#include "lcd_display.h"

LCDDisplay::LCDDisplay() : lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS), initialized(false) {
    last_blink_time = 0;
    blink_state = false;
    last_update_time = 0;
    last_mode = MODE_IDLE;
}

bool LCDDisplay::init() {
    lcd.init();
    lcd.backlight();
    
    // Test if LCD is responding
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Qbot Robot");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    
    delay(2000);
    initialized = true;
    return true;
}

void LCDDisplay::update() {
    if (!initialized) return;
    
    unsigned long current_time = millis();
    
    // Update eyes every 100ms or when mode changes
    if (current_time - last_update_time > 100 || robot_state.current_mode != last_mode) {
        updateEyes();
        last_update_time = current_time;
        last_mode = robot_state.current_mode;
    }
    
    // Handle blinking animation
    if (current_time - last_blink_time > 3000) { // Blink every 3 seconds
        blinkEyes();
        last_blink_time = current_time;
    }
}

void LCDDisplay::showBootMessage() {
    if (!initialized) return;
    
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("QBOT");
    lcd.setCursor(2, 1);
    lcd.print("INITIALIZING");
    delay(1000);
    
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("O O");
    lcd.setCursor(2, 1);
    lcd.print("Hello, I am");
    delay(1000);
}

void LCDDisplay::showError(String message) {
    if (!initialized) return;
    
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("X     X");
    lcd.setCursor(0, 1);
    lcd.print(message.substring(0, 16)); // Limit to LCD width
}

void LCDDisplay::updateEyes() {
    switch (robot_state.current_mode) {
        case MODE_IDLE:
            showIdleEyes();
            break;
        case MODE_VELOCITY:
            showVelocityEyes();
            break;
        case MODE_DIRECT:
            showDirectEyes();
            break;
        case MODE_ACTION:
            showActionEyes();
            break;
        case MODE_DIAGNOSTIC:
            showDiagnosticEyes();
            break;
    }
}

void LCDDisplay::showIdleEyes() {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("O     O");
    lcd.setCursor(4, 1);
    lcd.print("STANDBY");
}

void LCDDisplay::showVelocityEyes() {
    lcd.clear();
    lcd.setCursor(6, 0);
    if (robot_state.cmd_linear_vel > 0.01) {
        lcd.print(">     <");  // Moving forward
    } else if (robot_state.cmd_linear_vel < -0.01) {
        lcd.print("<     >");  // Moving backward
    } else if (abs(robot_state.cmd_angular_vel) > 0.01) {
        lcd.print("~     ~");  // Turning
    } else {
        lcd.print("O     O");  // Stopped
    }
    lcd.setCursor(3, 1);
    lcd.print("VELOCITY");
}

void LCDDisplay::showDirectEyes() {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("0 0");
    lcd.setCursor(4, 1);
    lcd.print("MANUAL");
}

void LCDDisplay::showActionEyes() {
    lcd.clear();
    lcd.setCursor(6, 0);
    switch (robot_state.action_state) {
        case ACTION_EXECUTING:
            lcd.print("^     ^");
            break;
        case ACTION_DONE:
            lcd.print("*     *");
            break;
        case ACTION_ERROR:
            lcd.print("X     X");
            break;
        default:
            lcd.print("-     -");
            break;
    }
    lcd.setCursor(4, 1);
    lcd.print("ACTION");
}

void LCDDisplay::showDiagnosticEyes() {
    lcd.clear();
    lcd.setCursor(6, 0);
    if (robot_state.heartbeat_ok ){//&& robot_state.tilt_ok) {
        lcd.print("O     -");  // Winking - all good
    } else {
        lcd.print("!     !");  // Alert - problem detected
    }
    lcd.setCursor(2, 1);
    lcd.print("DIAGNOSTICS");
}

void LCDDisplay::blinkEyes() {
    if (!blink_state) {
        lcd.setCursor(6, 0);
        lcd.print("-     -");
        blink_state = true;
        delay(150);
        updateEyes(); // Restore normal eyes
        blink_state = false;
    }
}

void lcd_update() {
    static uint32_t last = 0;
    if (millis() - last < 200) {
        return;
    }
    last = millis();

    // ...existing LCD draw logic...
}