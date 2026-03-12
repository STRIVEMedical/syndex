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
#include "admittance.h"
// #include "ODriveFlexCAN.hpp"

AdmittanceState admittance[NUM_JOINTS];
float last_loop_time_us = 0;


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
  // initJoints();
  // Serial.println("Setting input torqe (0.0)");
  // odrv0.setTorque(0.0);

  initAdmittance(&admittance[0], 0.5f, 2.0f, 0.087f, 5.0f);
  initAdmittance(&admittance[1], 0.5f, 2.0f, 0.087f, 5.0f);
  last_loop_time_us = micros();
}


void loop() {
    pumpEvents(can_intf); // Keep CAN callbacks firing

    float now = micros();
    float dt = (now - last_loop_time_us) / 1e6f;
    last_loop_time_us = now;
    if (dt > 0.05f) dt = 0.05f; // clamp if loop stalls

    readJointAnglesAndRaw();

    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);
        ODriveUserData* ud = (i == 0) ? &odrv0_user_data : &odrv1_user_data;
        ODriveCAN* odrv = (i == 0) ? &odrv0 : &odrv1;

        if (!ud->received_iq) continue;

        float angle_rad = j->angle * DEG_TO_RAD;
        float iq = ud->last_iq.Iq_Measured;

        float tau_ext = estimateExternalTorque(&admittance[i], iq, angle_rad);

        // Deadband: ignore tiny forces (noise floor)
        if (fabsf(tau_ext) < 0.1f) tau_ext = 0.0f;

        updateAdmittance(&admittance[i], tau_ext, dt);

        // Convert virtual velocity (rad/s) → motor turns/s
        float vel_cmd = admittance[i].vel / (2.0f * PI * admittance[i].gear_ratio);

        // Clamp for safety
        vel_cmd = constrain(vel_cmd, -2.0f, 2.0f);

        odrv.setVelocity(vel_cmd, 0.0f);
    }
}