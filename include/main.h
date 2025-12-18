// CAN Communication vvv

#include "ODriveFlexCAN.hpp"

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

bool setupCan();

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

void onCanMessage(const CanMsg& msg);

void setup();

void loop();

// USB Communication vvv 

#include <stdint.h>
#include <vector>
#include <cstring>

//Packet Constants
const uint16_t SYNC_BYTES = 0x7FFE;
const uint8_t MAX_PAYLOAD_SIZE = 255;

//Packet Types
enum packetType : uint8_t { //TO DO: UNCAPATALIZE ALL OF MY SHIT
  // Host -> Device (Commands)
  CMD_PING = 0x01,
  CMD_RESET_DEVICE = 0x02,
  CMD_SET_ODRIVE_STATE = 0x03,
  CMD_SET_JOINT_TARGETS = 0x04,
  CMD_REQUEST_TELEMETRY = 0x05,
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
struct PACKET_HEADER {
  uint16_t SYNC;
  uint8_t PACKET_TYPE;
  uint8_t PAYLOAD_SIZE;
};

//Payload Definitions
struct SET_ODRIVE_STATE_PAYLOAD {
  uint8_t JOINT_MASK;
  uint8_t ODRIVE_STATE;
};

struct JOINT_TARGET {
  uint16_t JN_TARGET_VELOCITY;
  uint16_t JN_TARGET_TORQUE_FF;
}; // TO DO: Represent as 16-bit floats

struct SET_JOINT_TARGETS_PAYLOAD {
  JOINT_TARGET JOINTS[7];
};

struct SET_JOINT_PARAMETER_PAYLOAD {
  uint8_t JOINT_MASK;
  uint8_t PARAMETER_ID;
  float VALUE;
};

struct JOINT_DATA {
  uint16_t JN_ANGLE;
  uint16_t JN_VELOCITY;
}; // TO DO: Represent as 16-bit floats

struct TELEM_JOINT_DATA_PAYLOAD {
  JOINT_DATA JOINTS[7];
};

struct TELEM_STATUS_PAYLOAD {
  uint8_t ARM_STATUS;
  uint8_t ODRIVE_FAULTS;
  uint8_t RESERVED[8];
};

//Packet Definitions
class PACKET {
public:
  PACKET_HEADER HEADER;
  uint8_t PAYLOAD[MAX_PAYLOAD_SIZE];
  uint16_t CHECKSUM;

  //Empty Packet Definition
  PACKET() {
    HEADER.SYNC = SYNC_BYTES;
    HEADER.PACKET_TYPE = 0;
    HEADER.PAYLOAD_SIZE = 0;
    CHECKSUM = 0;
    memset(PAYLOAD, 0, MAX_PAYLOAD_SIZE);
  }

  // Packet With Payload Definition
  template <typename T>
  PACKET(uint8_t TYPE, const T& DATA) {
    HEADER.SYNC = SYNC_BYTES;
    HEADER.PACKET_TYPE = TYPE;
    HEADER.PAYLOAD_SIZE = sizeof(T);
    memcpy(PAYLOAD, &DATA, sizeof(T));
    CHECKSUM = CALCULATE_CRC();
  }

  // Packet Without Payload Definition
  PACKET(uint8_t TYPE) {
    HEADER.SYNC = SYNC_BYTES;
    HEADER.PACKET_TYPE = TYPE;
    HEADER.PAYLOAD_SIZE = 0;
    CHECKSUM = CALCULATE_CRC();
  }

  // Packet to Byte Stream Conversion
  std::vector<uint8_t> SERIALIZE() {
    std::vector<uint8_t> BUFFER;
    uint8_t* HEADER_PTR = reinterpret_cast<uint8_t*>(&HEADER);
    BUFFER.insert(BUFFER.end(), HEADER_PTR, HEADER_PTR + sizeof(PACKET_HEADER));
    BUFFER.insert(BUFFER.end(), PAYLOAD, PAYLOAD + HEADER.PAYLOAD_SIZE);
    uint8_t* CRC_PTR = reinterpret_cast<uint8_t*>(&CHECKSUM);
    BUFFER.insert(BUFFER.end(), CRC_PTR, CRC_PTR + sizeof(uint16_t));
    return BUFFER;
  }
  
private:
  uint16_t CALCULATE_CRC() {
    return 0x0000; // TO DO: Define and implement CRC algorithm
  }
};