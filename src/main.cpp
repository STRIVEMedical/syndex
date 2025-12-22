#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"

/*
Main loop moved here from i2c.cpp so this file is the sketch entry
for runtime behavior. setup() remains implemented in odrive.cpp.
*/
void loop() {
  // Handle zero request from PC
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      zeroOffset[0] = 0;
      zeroOffset[1] = 0;
      turns[0] = turns[1] = 0;
      Serial.println("Zeroed!");
    }
  }

  // -------- SENSOR 0 --------
  tcaSelect(0);
  delayMicroseconds(500);
  uint16_t raw0 = readRawAS5600();
  float angle0 = computeAngle(0, raw0);

  // -------- SENSOR 1 --------
  tcaSelect(1);
  delayMicroseconds(500);
  uint16_t raw1 = readRawAS5600();
  float angle1 = computeAngle(1, raw1);

  // Output to host
  Serial.println("I2C READY");
  Serial.print("CH0: ");
  Serial.println(angle0, 2);

  //Prints number of Revolutions for encoder 0
  Serial.print("REV0: ");
  Serial.println(turns[0]);

  Serial.print("CH1: ");
  Serial.println(angle1, 2);

  //Prints number of Revolutions for encoder 1
  Serial.print("REV1: ");
  Serial.println(turns[1]);

  delay(5);
}
