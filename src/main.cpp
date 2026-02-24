#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "main.h"
// #include "ODriveFlexCAN.hpp"

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
  Serial.println("Setting input torqe (0.0)");
  odrv0.setTorque(0.0);
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

  // readJointAnglesAndRaw();

  // float angle0 = getJoint(0)->angle;
  // float angle1 = getJoint(1)->angle;

  bool joint1Home = getJoint(1)->is_homed;

  // Serial.print("CH0: ");
  // Serial.println(angle0, 2);



  // -------- SENSOR 0 SETUP --------
  // tcaSelect(0);
  // delayMicroseconds(500);
  // uint16_t raw0 = readRawAS5600();
  // float angle0 = computeAngle(0, raw0);


  // Keep CAN traffic serviced continuously (heartbeats, status frames, callbacks).
  // This should run every loop iteration and should not be blocked by delays.
  pumpEvents(can_intf);

  // --- Command update timer (how often we transmit torque command) ---
  static uint32_t lastCmdMs = 0;
  const uint32_t cmdPeriodMs = 10;   
  // 100 Hz updates

  // --- Direction toggle timer (how often we flip sign) ---
  static uint32_t lastFlipMs = 0;
  const uint32_t flipPeriodMs = 2000; 
  // change direction every 2 seconds

  // Base torque magnitude (Nm)
  const float torqueMag = 0.009f;

  // Current direction: +1 or -1
  static int dir = 1;

  // Flip direction every flipPeriodMs
  if (millis() - lastFlipMs >= flipPeriodMs) {
    lastFlipMs = millis();
    dir = -dir;
  }

  // Send torque command at fixed update rate
  if (millis() - lastCmdMs >= cmdPeriodMs) {
    lastCmdMs = millis();
    odrv0.setTorque(dir * torqueMag);
  }
}