#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"

void setup() {
  if (!initMultiOdrives()) {
    Serial.println("ODrive init failed — halting");
    while (1);
  }

  Buttons::setup();
  LED::setup();
  ONToggleLED(&LED::powerLED);
  ONToggleLED(&LED::dataLED);
}

/*
Main loop moved here from i2c.cpp so this file is the sketch entry
for runtime behavior. setup() remains implemented in odrive.cpp.
*/
void loop() {
  // buttonDetect(&buttonPins::powerButton);
  // buttonDetect(&buttonPins::autoHoming);
  // buttonDetect(&buttonPins::triggerButton);
  // buttonDetect(&buttonPins::toolSelect);

  // Handle zero request from PC
  // if (Serial.available()) {
  //   char c = Serial.read();
  //   if (c == 'z') {
  //     for(int i = 0; i < 7; i++) {
  //     zeroOffset[i] = 0;
  //     turns[i] = 0;
  //     }
  //     Serial.println("Zeroed!");
  //   }
  // }

  // -------- SENSOR 0 SETUP --------
  // tcaSelect(0);
  // delayMicroseconds(500);
  // uint16_t raw0 = readRawAS5600();
  // float angle0 = computeAngle(0, raw0);

  // -------- SENSOR 1 SETUP --------
  // tcaSelect(1);
  // delayMicroseconds(200);
  // uint16_t raw1 = readRawAS5600();
  // float angle1 = computeAngle(1, raw1);

  // -------- SENSOR 2 SETUP --------
  // tcaSelect(2);
  // delayMicroseconds(500);
  // uint16_t raw2 = readRawAS5600();
  // float angle2 = computeAngle(2, raw2);

  // // -------- SENSOR 3 SETUP --------
  // tcaSelect(3);
  // delayMicroseconds(500);
  // uint16_t raw3 = readRawAS5600();
  // float angle3 = computeAngle(3, raw3);

  // // -------- SENSOR 4 SETUP --------
  // tcaSelect(4);
  // delayMicroseconds(500);
  // uint16_t raw4 = readRawAS5600();
  // float angle4 = computeAngle(4, raw4);

  // // -------- SENSOR 5 SETUP --------
  // tcaSelect(5);
  // delayMicroseconds(500);
  // uint16_t raw5 = readRawAS5600();
  // float angle5 = computeAngle(5, raw5);

  // // -------- SENSOR 6 SETUP --------
  // tcaSelect(6);
  // delayMicroseconds(500);
  // uint16_t raw6 = readRawAS5600();
  // float angle6 = computeAngle(6, raw6);

  // // -------- SENSOR 7 SETUP --------
  // tcaSelect(7);
  // delayMicroseconds(500);
  // uint16_t raw7 = readRawAS5600();
  // float angle7 = computeAngle(7, raw7);

  // READING SENSOR 0
  // Serial.println("I2C READY");
  // Serial.print("CH0: ");
  // Serial.println(angle0, 2);

  // Prints number of Revolutions for encoder 0
  // Serial.print("REV0: ");
  // Serial.println(turns[0]);

  // READING SENSOR 1
  // Serial.print("CH1: ");
  // Serial.println(angle1, 2);

  //Prints number of Revolutions for encoder 1  
  // Serial.print("REV1: ");
  // Serial.println(turns[1]);

  // Reading Sensor 2
  // Serial.print("CH2: ");
  // Serial.println(angle2, 2);

  // //Prints number of Revolutions for encoder 2
  // Serial.print("REV2: ");
  // Serial.println(turns[2]);

  // // Reading Sensor 3
  // Serial.print("CH3: ");
  // Serial.println(angle3, 3);

  // //Prints number of Revolutions for encoder 3
  // Serial.print("REV3: ");
  // Serial.println(turns[3]);

  // // Reading Sensor 4
  // Serial.print("CH4: ");
  // Serial.println(angle4, 4);

  // //Prints number of Revolutions for encoder 4
  // Serial.print("REV4: ");
  // Serial.println(turns[4]);

  // // Reading Sensor 5
  // Serial.print("CH5: ");
  // Serial.println(angle5, 5);

  // //Prints number of Revolutions for encoder 5
  // Serial.print("REV5: ");
  // Serial.println(turns[5]);

  // // Reading Sensor 6
  // Serial.print("CH6: ");
  // Serial.println(angle6, 6);

  // //Prints number of Revolutions for encoder 6
  // Serial.print("REV6: ");
  // Serial.println(turns[6]);

  // // Reading Sensor 7
  // Serial.print("CH7: ");
  // Serial.println(angle7, 7);

  // //Prints number of Revolutions for encoder 7
  // Serial.print("REV7: ");
  // Serial.println(turns[7]);
}

// void loop() {
//   tcaSelect(0);
//   setupI2C();

//   uint16_t angle = readRawAS5600();

//   if (angle != 0xFFFF) {
//     float degrees = angle * 360.0 / 4096.0;
//     Serial.print("Raw: ");
//     Serial.print(angle);
//     Serial.print("  Degrees: ");
//     Serial.println(degrees, 2);
//   } else {
//     Serial.println("Read failed");
//   }

//   delay(20);
// }