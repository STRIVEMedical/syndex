#ifndef USB_H
#define USB_H


// USB Communication vvv 

#include <stdint.h>
#include <vector>
#include <cstring>
#include <cstdint>

#include "comms.h"

//Packet Constants
// Serialized as 0x7F 0xFE on little-endian MCUs to match host framing.
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
  CMD_CONFIRM_HOME = 0x09,  // Operator has positioned arm — latch current encoder pos as zero
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
  float jnTargetVelocity;
  float jnTargetTorqueFF;
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
  float jnAngle;
  float jnVelocity;
}; // Represented as 16-bit floats in USB.cpp (Uses Float16 Conversion Functions)
  //Converted to unsigned16 before sending, converted back to float16 after recieving

struct telemStatusPayload {
  uint8_t armStatus;
  uint8_t odriveFaults;
  uint8_t reserved[8];
};

struct telemJointDataPayload {
  jointData joints[7];
  uint8_t triggerPressed;
};

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
  
  //CRC-16/ARC Checksum Calculation Algorithm
  uint16_t calculateCRC();

private:
  uint16_t updateCRC(uint16_t crc, uint8_t data);
};

//16-bit Float Conversion Functions (For Kinematics Payloads)
uint16_t float16ToUnsigned16(float value);
float unsigned16ToFloat16(uint16_t bits);

// USB packet I/O functions
void pollSerialPackets(); // Poll Serial for incoming bytes and feed parser. Call this often from loop() or stateUpdate().
void processIncomingPackets(); // Process any available incoming packets by dispatching to handlers.

// Helper functions to send specific packets
void sendCmdPong(); // Helper to send RESP_PONG packet.
void sendCmdAck(); // Send RESP_ACK
void sendCmdNack(); // Send RESP_NACK
void sendTelemJointData(const telemJointDataPayload& payload); // Send joint telemetry
void sendTelemStatus(const telemStatusPayload& payload); // Send status telemetry
void sendLogMessage(const char* message); // Send log message
void sendErrorMessage(const char* message); // Send error message

// Loads one joint's telemetry values into the payload entry.
void buildTelemJointPayload(telemJointDataPayload& payload, int jointID, float angleDeg, float velocityDegPerSec);

// External flag for ping received
extern volatile bool pingReceived;
// One-shot flag set on each CMD_PING and consumed by state machine transition logic.
extern volatile bool pingEventPending;
// One-shot flag set when CMD_CONFIRM_HOME arrives; consumed by the HOMING state to latch zero.
extern volatile bool confirmHomePending;
// One-shot flag set when CMD_START_HOMING arrives; consumed by the READY state to drive arm to home pos.
extern volatile bool moveToHomePending;


#endif // USB_H