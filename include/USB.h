#ifndef USB_H
#define USB_H


// USB Communication vvv 

#include <stdint.h>
#include <vector>
#include <cstring>
#include <cstdint>

#include "comms.h"

//16-bit Float Conversion Functions (For Kinematics Payloads)
uint16_t float16ToUnsigned16(float value);
float unsigned16ToFloat16(uint16_t bits);

//Packet Constants
const uint16_t SYNC_BYTES = 0x7FFE;
const uint8_t MAX_PAYLOAD_SIZE = 255;

//Packet Types
enum packetType : uint8_t {
  // Host -> Device (Commands)
  CMD_PING = 0x01,
  CMD_RESET_DEVICE = 0x02,
  CMD_SET_ODRIVE_STATE = 0x03,
  CMD_SET_JOINT_TARGETS = 0x04,
  CMD_REQUEST_TELEM = 0x05, // TELEM = Telemetry
  CMD_START_HOMING = 0x06,
  CMD_SET_JOINT_PARAMETER = 0x07,
  CMD_ESTOP = 0x08,
  // Device -> Host (Telemetry & Responses)
  RESP_PONG = 0x81,
  RESP_ACK = 0x82,
  RESP_NACK = 0x83,
  TELEM_JOINT_DATA = 0x84,
  TELEM_STATUS = 0x85,
  LOG_MESSAGE = 0x86,
  ERROR_MESSAGE = 0xF0,
};

//Packet Header Definition
struct packetHeader {
  uint16_t sync;
  uint8_t packetType;
  uint8_t payloadSize;
};

//Payload Definitions
struct setOdriveStatePayload {
  uint8_t jointMask;
  uint8_t odriveState;
};

struct jointTarget {
  uint16_t jnTargetVelocity;
  uint16_t jnTargetTorqueFF;
}; // Represeted as 16-bit floats in USB.cpp (Uses Float16 Conversion Functions).
   //Converted to unsigned16 before sending, converted back to float16 after recieving

struct setJointTargetsPayload {
  jointTarget joints[7];
};

struct setJointParameterPayload {
  uint8_t jointMask;
  uint8_t parameterID;
  float value;
};

struct jointData {
  uint16_t jnAngle;
  uint16_t jnVelocity;
}; // Represented as 16-bit floats in USB.cpp (Uses Float16 Conversion Functions)
  //Converted to unsigned16 before sending, converted back to float16 after recieving

struct telemStatusPayload {
  uint8_t armStatus;
  uint8_t odriveFaults;
  uint8_t reserved[8];
};

struct telemJointDataPayload {
  jointData joints[7];
};

// Loads one joint's telemetry values into the payload entry.
void buildTelemJointPayload(telemJointDataPayload& payload, int jointID, float angleDeg, float velocityDegPerSec);

//Packet Definitions
class packet {
public:
  packetHeader header;
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint16_t checksum;

  //Empty Packet Definition
  packet();

  // Packet With Payload Definition
  template <typename T>
  packet(uint8_t type, const T& data) {
    header.sync = SYNC_BYTES;
    header.packetType = type;
    header.payloadSize = sizeof(T);
    memcpy(payload, &data, sizeof(T));
    checksum = calculateCRC();
  }

  // Packet Without Payload Definition
  packet(uint8_t type);

  // Packet to Byte Stream Conversion (Byte stream = final data to be sent)
  std::vector<uint8_t> serialize();
  
private:
  //CRC-16/ARC Checksum Calculation Algorithm
  uint16_t calculateCRC();
  uint16_t updateCRC(uint16_t crc, uint8_t data);
};

#endif // USB_H