#include <Arduino.h>
#include "com.h"

void setup() {
  initCom(9600); // Initialize communication with the specified baud rate
}

void loop() {
  readCommand();    // Read the command from Serial
  sendFeedback();   // Send feedback based on the received command
  delay(500);       // Delay for 500ms before the next iteration
}
