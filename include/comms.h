// comms.h
// Unified communication-layer declarations (CAN + I2C)

#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <Wire.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>

/* CAN Settings ----------------------------------------------------*/
#define CAN_BAUDRATE 250000

// Node IDs for each ODrive
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2

/* I2C Settings ----------------------------------------------------*/
#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36
#define ANGLE_HIGH 0x0C
#define ANGLE_LOW 0x0F
#define NUM_ENCODERS 3

// I2C encoder state
extern float zeroOffset[NUM_ENCODERS];
extern long turns[NUM_ENCODERS];
extern int lastRaw[NUM_ENCODERS];

// CAN interface + ODrive instances
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
extern ODriveCAN odrv0;
extern ODriveCAN odrv1;
extern ODriveCAN odrv2;
extern ODriveCAN* odrives[3];

// CAN setup and callbacks
bool setupCan();
void onCanMessage(const CAN_message_t& msg);
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void onCurrents(Get_Iq_msg_t& msg, void* user_data);

// System-level communication init
bool initCommunications();

// I2C utility functions
void tcaSelect(uint8_t ch);
uint16_t readRawAS5600();
float computeAngle(int sensorID, uint16_t raw);
void setupI2C();

#endif // COMMS_H
