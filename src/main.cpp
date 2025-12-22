#include <Arduino.h>
#include "ODriveCAN.h"
#include "main.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

/* CAN Settings ----------------------------------------------------*/
#define CAN_BAUDRATE 250000

// Node IDs for each ODrive
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

struct ODriveStatus; // Teensy compile hack

/* FlexCAN Interface -----------------------------------------------*/
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
void onCanMessage(const CanMsg& msg);

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

/* ODrive Instances ------------------------------------------------*/
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);

// List of all drives for message routing
ODriveCAN* odrives[] = { &odrv0, &odrv1 };

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

/*
Stores incoming heartbeat message and marks drive as detected on CAN bus
*/
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_heartbeat = msg;
  ud->received_heartbeat = true;
}

/*
Records encoder position and velocity for the associated ODrive
*/
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_feedback = msg;
  ud->received_feedback = true;
}

/*
Routes every incoming CAN frame to each ODriveCAN instance so each can decide
whether the frame matches its configured node ID
*/
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives)
    onReceive(msg, *odrive);
}

/*
- Initializes Serial for debugging
- registers ODrive callbacks
- initializes CAN hardware
- waits for heartbeat from both ODrives
- transitions both ODrives into closed-loop control
*/
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Starting Dual ODrive CAN Demo");

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

  // Helper lambda: enable closed-loop mode
  auto enable_closed_loop = [&](ODriveCAN &odrv, ODriveUserData &data) {
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
  };

  Serial.println("Enabling Closed Loop Control...");
  enable_closed_loop(odrv0, odrv0_user_data);
  enable_closed_loop(odrv1, odrv1_user_data);
  Serial.println("Both ODrives Running!");
}