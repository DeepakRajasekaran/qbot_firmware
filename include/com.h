#ifndef COM_H
#define COM_H

#include <Arduino.h>

void initCom(unsigned long baudRate);

void readCommand();

void sendFeedback();

void parseCommand();

#endif