// odrive functionality

#include <Arduino.h>
#include "ODriveCAN.h"
#include "odrive.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include "i2c.h"

// Define global instances declared as extern in odrive.h
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = { &odrv0, &odrv1 };
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

/* =========================
 * CAN SETUP
 * ========================= */

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

/* =========================
 * ODRIVE STATE CONTROL
 * ========================= */

void enable_closed_loop(ODriveCAN &odrv, ODriveUserData &data) {
  while (data.last_heartbeat.Axis_State !=
         ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    delay(1);
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; i++) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
}

/* =========================
 * CAN CALLBACKS
 * ========================= */

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_heartbeat = msg;
  ud->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_feedback = msg;
  ud->received_feedback = true;
}

void onCanMessage(const CAN_message_t& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

/* =========================
 * GLOBAL / SYSTEM INIT
 * ========================= */

bool initOdriveSystem() {
  Serial.begin(115200);
  delay(200);

  if (!setupCan()) {
    Serial.println("CAN init failed");
    return false;
  }

  // Initialize I2C sensors
  setupI2C();

  Serial.println("CAN & I2C READY");
  return true;
}

/* =========================
 * PER-ODRIVE INIT (GENERAL)
 * ========================= */

bool initOdrive(ODriveCAN &odrv, ODriveUserData &data) {
  // Register callbacks
  odrv.onStatus(onHeartbeat, &data);
  odrv.onFeedback(onFeedback, &data);

  // Wait for heartbeat
  Serial.println("Waiting for ODrive...");
  while (!data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("ODrive Found!");

  // Enable closed-loop control
  Serial.println("Enabling Closed Loop Control...");
  enable_closed_loop(odrv, data);
  Serial.println("ODrive Running!");
  return true;
}

/* =========================
 * INIT MULTIPLE ODRIVES
 * ========================= */

bool initMultiOdrives() {
  if (!initOdriveSystem()) {
    return false;
  }

  if (!initOdrive(odrv0, odrv0_user_data)) {
    return false;
  }
  
  if (!initOdrive(odrv1, odrv1_user_data)) {
    return false;
  }

  Serial.println("All ODrives Running!");
  return true;
}

/* =========================
 * CONFIGURE ODRIVE (should only need to be done once)
 * ========================= */

// bool configureOdrive(ODriveCAN &odrv, ODriveUserData &data) {
//   // Motor type is typically pre-configured on the ODrive via web interface
//   // No CAN command exists to change motor type....
  
//   // Proceed with encoder offset calibration if needed
//   Serial.println("Configuring ODrive...");
//   odrv.setState(ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
  
//   // Wait for calibration to complete
//   while (data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
//     pumpEvents(can_intf);
//     delay(10);
//   }
  
//   return true;
// }


