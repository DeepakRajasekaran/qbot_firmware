#ifndef COM_H
#define COM_H

#include <Arduino.h>

extern String command; // Declare as extern

void initCom(uint16_t baudRate);
void readCommand();
void sendFeedback();

#endif // COM_H