#include "com.h"
#include "globals.h"
#include "imu.h"

#define RX_TIMEOUT_MS 50

void initCom(unsigned long baudRate) {
    Serial.begin(baudRate);
    Serial.setTimeout(50); // 50ms timeout for readString
}

void readCommand() {
    while (Serial.available()) {
        char c = Serial.read();
        command += c;

        // Check for delimiter
        if (c == CMD_DELIMITER) {
            command.trim(); // Remove whitespace/newlines

            // Validate command starts with known keyword
            if (isValidCommand(command)) {
                command = command;
                parseCommand();
                robot_state.last_command_time = millis();
                robot_state.heartbeat_ok = true;
            } else {
                // Invalid or partial command, discard
                Serial.println("Invalid command discarded: " + command);
            }

            command = ""; // Clear buffer for next command
        }
    }

    // Heartbeat timeout failsafe
    if (millis() - robot_state.last_command_time > HEARTBEAT_TIMEOUT_MS) {
        robot_state.heartbeat_ok = false;
        robot_state.cmd_linear_vel = 0.0;
        robot_state.cmd_angular_vel = 0.0;
        robot_state.cmd_pwm_left = 0.0;
        robot_state.cmd_pwm_right = 0.0;
    }
}

// Function to validate command starts with known keyword
bool isValidCommand(const String &cmd) {
    String temp = cmd;
    temp.toUpperCase();
    return temp.startsWith("VEL ") ||
           temp.startsWith("DRV ") ||
           temp.startsWith("ACT ") ||
           temp.startsWith("MODE ") ||
           temp.startsWith("RESET") ||
           temp.startsWith("STOP");
}

void sendFeedback() {
    // Send structured feedback packet
    Serial.print("<ODOM,");
    Serial.print("x:"); Serial.print(robot_state.x, 4);
    Serial.print(",y:"); Serial.print(robot_state.y, 4);
    Serial.print(",th:"); Serial.print(robot_state.theta, 4);
    Serial.print(",v:"); Serial.print(robot_state.linear_vel, 3);
    Serial.print(",w:"); Serial.print(robot_state.angular_vel, 3);
    
    Serial.print(",MODE:"); Serial.print(robot_state.current_mode);
    Serial.print(",ACTION:"); Serial.print(robot_state.action_state);
    Serial.print(",HB:"); Serial.print(robot_state.heartbeat_ok ? 1 : 0);

    // Append IMU data to feedback packet
    Serial.print(",IMU_AX:"); Serial.print(imu_ok ? imu_ax : 0, 4);
    Serial.print(",IMU_AY:"); Serial.print(imu_ok ? imu_ay : 0, 4);
    Serial.print(",IMU_AZ:"); Serial.print(imu_ok ? imu_az : 0, 4);

    Serial.print(",IMU_GX:"); Serial.print(imu_ok ? imu_gx : 0, 4);
    Serial.print(",IMU_GY:"); Serial.print(imu_ok ? imu_gy : 0, 4);
    Serial.print(",IMU_GZ:"); Serial.print(imu_ok ? imu_gz : 0, 4);

    Serial.print(",IMU_ROLL:"); Serial.print(imu_ok ? imu_roll : 0, 4);
    Serial.print(",IMU_PITCH:"); Serial.print(imu_ok ? imu_pitch : 0, 4);
    Serial.print(",IMU_YAW:"); Serial.print(imu_ok ? imu_yaw : 0, 4);

    Serial.println(">");
}

void parseCommand() {
    // Parse commands: VEL v w | DRV pwmL pwmR | ACT action_id | MODE mode_id
    command.toUpperCase();
    
    if (command.startsWith("VEL ")) {
        // Parse velocity command: VEL v w
        int firstSpace = command.indexOf(' ', 4);
        if (firstSpace > 0) {
            robot_state.cmd_linear_vel = command.substring(4, firstSpace).toFloat();
            robot_state.cmd_angular_vel = command.substring(firstSpace + 1).toFloat();
            robot_state.current_mode = MODE_VELOCITY;
        }
    }
    else if (command.startsWith("DRV ")) {
        // Parse direct drive command: DRV pwmL pwmR
        int firstSpace = command.indexOf(' ', 4);
        if (firstSpace > 0) {
            robot_state.cmd_pwm_left = command.substring(4, firstSpace).toFloat();
            robot_state.cmd_pwm_right = command.substring(firstSpace + 1).toFloat();
            robot_state.current_mode = MODE_DIRECT;
        }
    }
    else if (command.startsWith("ACT ")) {
        // Parse action command: ACT action_id
        robot_state.current_action_id = command.substring(4).toInt();
        robot_state.current_mode = MODE_ACTION;
        robot_state.action_state = ACTION_EXECUTING;
    }
    else if (command.startsWith("MODE ")) {
        // Parse mode command: MODE mode_id
        uint8_t new_mode = command.substring(5).toInt();
        if (new_mode >= MODE_IDLE && new_mode <= MODE_DIAGNOSTIC) {
            robot_state.current_mode = (RobotMode)new_mode;
        }
    }
    else if (command.startsWith("RESET")) {
        // Reset odometry
        robot_state.x = 0.0;
        robot_state.y = 0.0;
        robot_state.theta = 0.0;
    }
    else if (command.startsWith("STOP")) {
        // Emergency stop
        robot_state.cmd_linear_vel = 0.0;
        robot_state.cmd_angular_vel = 0.0;
        robot_state.cmd_pwm_left = 0.0;
        robot_state.cmd_pwm_right = 0.0;
        robot_state.current_mode = MODE_IDLE;
    }
}

static uint32_t last_rx_time = 0;
static uint8_t rx_index = 0; // Move here if it was inside another function

void byte_receive_handler(uint8_t byte) {
    uint32_t now = millis();
    if (now - last_rx_time > RX_TIMEOUT_MS) {
        rx_index = 0;
    }
    last_rx_time = now;

    // ...existing buffer handling and parsing logic...
}

void process_packet(...) {
    // ...existing code...
    g_last_cmd_time = millis();
    // ...existing code...
}

void build_feedback_packet() {
    // ...existing code...

    // Append IMU data
    if (imu_ok) {
        // Accelerometer (raw int16)
        tx_buffer[idx++] = highByte(imu_ax);
        tx_buffer[idx++] = lowByte(imu_ax);

        tx_buffer[idx++] = highByte(imu_ay);
        tx_buffer[idx++] = lowByte(imu_ay);

        tx_buffer[idx++] = highByte(imu_az);
        tx_buffer[idx++] = lowByte(imu_az);

        // Gyroscope (raw int16)
        tx_buffer[idx++] = highByte(imu_gx);
        tx_buffer[idx++] = lowByte(imu_gx);

        tx_buffer[idx++] = highByte(imu_gy);
        tx_buffer[idx++] = lowByte(imu_gy);

        tx_buffer[idx++] = highByte(imu_gz);
        tx_buffer[idx++] = lowByte(imu_gz);
    } else {
        // Fill with zeros if IMU is not present
        for (uint8_t i = 0; i < 12; i++) {
            tx_buffer[idx++] = 0;
        }
    }

    // ...existing code...
}