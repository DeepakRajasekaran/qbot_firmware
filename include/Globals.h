#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "com.h"


#endif // GLOBALS_H

/*
    Dependencies
    electronicCats mpu6050 or adafruit_mpu6050
    PID
    
    code architecture

    main.ino

    globals.h
        - will contain data needs to be accessable by all modules

    - stateManager.h
        - stateSetup - define the state machine
        - stateLoop - run the state machine
        - stateFeedback - get the feedback from all modules and send it to the serial
    - drive.h
        - driveSetup - define left and right drives, setup encoders as well as pid
        - runDrive - need pid loop too run each motor in the given command rpm 
        - getfeedback - get the feedback from the encoders convert to rpm
    - imu
        - imuSetup
        - getImuData
    - battery
        - batterySetup - define the battery read pin
        - getBatteryData - read the battery voltage and convert to percentage
    - lcd
        - lcdSetup - define lcd
        - setLCD - set the lcd state depends upon the machine state
    - communication
        - serial
            - init - init the serial port
            - readCommand - parse the data we got and send it to the right module
            - sendFeedback - encode the from all modules and send it to the serial
        - i2c
            - imu_com
            - lcd_com

------------------------------------------------------------------------------------------------------------------

TASKS:

        1. commmunication - implement the serial communication module for both sides ros2 and arduino
            1.1 get data from pc and convert it into needed data type and write it in globals
            1.2 get data from globals and convert it into needed data type and send it to pc
            
        2. globals.h - define the data structure for the drive, imu, battery, 
           command and feedback structures

        3. stateManager - implement state machine, state transitions and state feedbacks

        4. drive - implement the drive module, setup the encoders and pid, run the pid loop

        5. imu - implement the imu module, setup the imu and get the data

        6. battery - implement the battery module, setup the battery and get the data

        7. lcd - implement the lcd module, setup the lcd and set the state

        8. test everything

-------------------------------------------------------------------------------------------------------------------

    communication - serial architecture

    string command =
*/