// Shared wire protocol used by BOTH the Pi base program and the ESP32-S3 handle firmware:
// packet IDs, payload structs, framing, and CRC-16/ARC. Ported from V1's USB.h/USB.cpp with the
// motor/joint/ODrive items removed. Header-only: each build just adds shared/ to its include path.
// The receive-side byte-stream parser for the Bluetooth serial Unity link is in serial_framing.h.
//
// For: Both base + handle
//
// Keep this file free of Arduino/Teensy/Linux-specific includes and of C++17-only features:
// the ESP32 Arduino core builds as gnu++11, the Pi builds as C++17.

#ifndef SYNDEX_MESSAGES_H
#define SYNDEX_MESSAGES_H

#include <stdint.h>
#include <cstring>
#include <vector>

//Packet Constants
// Serialized as 0x7F 0xFE on little-endian MCUs to match host framing.
const uint16_t SYNC_BYTES = 0x7FFE;
const uint8_t MAX_PAYLOAD_SIZE = 255;

//Packet Types
enum packetType : uint8_t {
  // Host -> Device (Commands)
  CMD_PING = 0x01,
  CMD_RESET_DEVICE = 0x02,
  CMD_REQUEST_TELEM = 0x05, // TELEM = Telemetry
  // Device -> Host (Telemetry & Responses)
  RESP_PONG = 0x81,
  RESP_ACK = 0x82,
  RESP_NACK = 0x83,
  TELEM_STATUS = 0x85,
  LOG_MESSAGE = 0x86,
  ERROR_MESSAGE = 0xF0,
};

// Retired V1 packet IDs (motor/homing-era). Do not reuse these numbers for anything else while
// Unity or old tools may still send them:
//   0x03 CMD_SET_ODRIVE_STATE   0x04 CMD_SET_JOINT_TARGETS   0x06 CMD_START_HOMING
//   0x07 CMD_SET_JOINT_PARAMETER   0x08 CMD_ESTOP   0x09 CMD_CONFIRM_HOME   0x84 TELEM_JOINT_DATA

// TODO(V2): define the V2 messages here (IDs + payload structs): handle -> base state (handle ID,
// orientation, gyro/accel, trigger, dock state, sequence/timestamp), base -> Unity pose telemetry,
// Unity -> handle vibration command, heartbeats. See Firmware Plan items 7.4, 10.5, 11 and 13.3.

//Packet Header Definition
struct packetHeader {
  uint16_t sync;
  uint8_t packetType;
  uint8_t payloadSize;
};

//Payload Definitions
// The old odriveFaults byte (offset 1) is now reserved; the payload stays 10 bytes.
struct telemStatusPayload {
  uint8_t armStatus;
  uint8_t reserved[9];
};

// Codes 0x03-0x06 were V1 homing/assist states and are retired.
enum armStatusCode : uint8_t {
  ARM_STATUS_BOOTUP = 0x00,
  ARM_STATUS_IDLE = 0x01,
  ARM_STATUS_CONNECTED = 0x02,
  ARM_STATUS_POWERING_OFF = 0x07,
  ARM_STATUS_ERROR = 0x08,
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
    std::memset(payload, 0, MAX_PAYLOAD_SIZE);
  }

  // Packet With Payload Definition
  template <typename T>
  packet(uint8_t type, const T& data) {
    header.sync = SYNC_BYTES;
    header.packetType = type;
    header.payloadSize = sizeof(T);
    std::memcpy(payload, &data, sizeof(T));
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
    //MCUs are little-endian
    //Eg. For 0x7FFE
    //Parser should expect 0xFE then 0x7F
    std::vector<uint8_t> buffer;
    checksum = calculateCRC();

    // sync in little-endian
    buffer.push_back(0x7F);
    buffer.push_back(0xFE);

    buffer.push_back(header.packetType);
    buffer.push_back(header.payloadSize);

    buffer.insert(buffer.end(), payload, payload + header.payloadSize);

    buffer.push_back(checksum & 0xFF);
    buffer.push_back((checksum >> 8) & 0xFF);

    return buffer;
  }

  //CRC-16/ARC Checksum Calculation Algorithm
  uint16_t calculateCRC() {
    uint16_t crc = 0x0000;
    crc = updateCRC(crc, header.packetType);
    crc = updateCRC(crc, header.payloadSize);
    for (uint8_t i = 0; i < header.payloadSize; ++i) {
      crc = updateCRC(crc, payload[i]);
    }
    return crc;
  }

private:
  uint16_t updateCRC(uint16_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
    return crc;
  }
};

#endif // SYNDEX_MESSAGES_H
