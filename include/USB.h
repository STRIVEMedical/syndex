#ifndef USB_H
#define USB_H


// USB Communication vvv 

#include <stdint.h>
#include <vector>
#include <cstring>
#include <cstdint>

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

void buildTelemJointPayload(telemJointDataPayload& payload, int sensorID, uint16_t raw) {
    float ang = computeAngle(sensorID, raw);
    payload.joints[sensorID].jnAngle = ang;
}; //Function that loads the angle from the encoder and puts it into the payload (for a single encoder)

struct telemJointDataPayload {
  jointData joints[7];
};

struct telemStatusPayload {
  uint8_t armStatus;
  uint8_t odriveFaults;
  uint8_t reserved[8];
};

//Packet Definitions
class packet {
public:
  packetHeader header;
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint16_t checksum;

  //Empty Packet Definition
  packet() {
    header.sync = SYNC_BYTES;
    header.packetType = 0;
    header.payloadSize = 0;
    checksum = 0;
    memset(payload, 0, MAX_PAYLOAD_SIZE);
  }

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
  packet(uint8_t type) {
    header.sync = SYNC_BYTES;
    header.packetType = type;
    header.payloadSize = 0;
    checksum = calculateCRC();
  }

  // Packet to Byte Stream Conversion (Byte stream = final data to be sent)
  std::vector<uint8_t> serialize() {
    std::vector<uint8_t> buffer;
    //add header bytes to byte stream
    uint8_t* headerPtr = reinterpret_cast<uint8_t*>(&header);
    buffer.insert(buffer.end(), headerPtr, headerPtr + sizeof(packetHeader));
    //append payload bytes to byte stream
    buffer.insert(buffer.end(), payload, payload + header.payloadSize);
    //calculate CRC
    checksum = calculateCRC();
    //append CRC to byte stream in little-endian order
    buffer.push_back(checksum & 0xFF);
    buffer.push_back((checksum >> 8) & 0xFF);
    return buffer;
  }
  
private:
  //CRC-16/ARC Checksum Calculation Algorithm
  uint16_t calculateCRC() {
    uint16_t crc = 0x0000;
    crc = updateCRC(crc, header.packetType); //CRC for packetType
    crc = updateCRC(crc, header.payloadSize); //CRC for payloadSize
    for (uint8_t i = 0; i < header.payloadSize; ++i) { 
      crc = updateCRC(crc, payload[i]);
    } //CRC for all payload bytes
    return crc; //return final CRC value (0x0000 == packet is valid)
  }

  uint16_t updateCRC(uint16_t crc, uint8_t data) {
    crc ^= data; //XOR with current CRC
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1) //If LSB is 1, shift right and XOR with polynomial
        crc = (crc >> 1) ^ 0xA001;
      else //If LSB is 0, simply shift right
        crc >>= 1;
    }
    return crc;
  }
};

#endif // USB_H