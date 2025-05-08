#include "com.h"

String command; // Define the global variable here

void initCom(uint16_t baudRate)
{
    Serial.begin(baudRate);
}

void readCommand()
{
    command = Serial.available() ? Serial.readString() : command;
}

void sendFeedback()
{
    Serial.println("received command: " + command);
}

void parseCommand()
{
    return;
}