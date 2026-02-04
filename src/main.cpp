#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"


void setup() {
  if (!initMultiOdrives()) {
    Serial.println("ODrive init failed — halting");
    while (1);
  }

  Buttons::setup();
  LED::setup();
  ONToggleLED(&LED::powerLED);
  ONToggleLED(&LED::dataLED);

  // Initialize joints with hardware mapping
  initJoints();
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

  readJointAnglesAndRaw();

  float angle0 = getJoint(0)->angle;
  float angle1 = getJoint(1)->angle;

  Serial.print("CH0: ");
  Serial.println(angle0, 2);



  // -------- SENSOR 0 SETUP --------
  // tcaSelect(0);
  // delayMicroseconds(500);
  // uint16_t raw0 = readRawAS5600();
  // float angle0 = computeAngle(0, raw0);

}
