#include <Arduino.h>
#include "Globals.h"

void setup() {
  Serial.begin(9600);
  driveSetup();
}

void loop() {
  // a = digitalRead(M1_ENC_1);
  // b = digitalRead(M1_ENC_2);
  // Serial.print(int(b)*5);
  // Serial.print(">ticks:");
  // Serial.println(ticks);
  // delay(1000);
}
