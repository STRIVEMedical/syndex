#ifndef ODRIVE_TEENSY_CAN_H
#define ODRIVE_TEENSY_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"

// ---------------------- Configuration ----------------------

#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0

// ---------------------- Global Declarations ----------------------

// CAN interface instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// ODrive instance
extern ODriveCAN odrv0;

// Struct to hold user data for each ODrive
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback;
};

extern ODriveUserData odrv0_user_data;

// ---------------------- Function Prototypes ----------------------

// Initializes the CAN bus
bool setupCan();

// Called every time a Heartbeat message arrives
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

// Called every time a feedback message arrives
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg);

#endif // ODRIVE_TEENSY_CAN_H
