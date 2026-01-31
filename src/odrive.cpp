//odrive functionality 

#include <Arduino.h>
#include "ODriveCAN.h"
#include "odrive.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include "i2c.h"

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

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


void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives)
    onReceive(msg, *odrive);
}


void setupODrive() {
  Serial.begin(115200);
  delay(200);

  // Register callbacks
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv0.onFeedback(onFeedback, &odrv0_user_data);

  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);

  // Initialize CAN
  if (!setupCan()) {
    Serial.println("CAN init failed");
    while (1);
  }

  // Wait for both drives
  Serial.println("Waiting for both ODrives...");
  while (!odrv0_user_data.received_heartbeat ||
         !odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("Both ODrives Found!");
  Serial.println("CAN READY");

  // Initialize I2C sensors (moved loop is in main.cpp)
  setupI2C();

  Serial.println("Enabling Closed Loop Control...");
  enable_closed_loop(odrv0, odrv0_user_data);
  enable_closed_loop(odrv1, odrv1_user_data);
  Serial.println("Both ODrives Running!");
}