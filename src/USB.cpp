#include <stdint.h>
#include <vector>
#include <cstring>
#include <cmath>
#include "USB.h"

// File-local helpers used only inside the USB module.
static bool getNextPacket(packet& outPacket);
// static void sendPacket(packet& pkt);
static void sendPacket(const packet& pkt);

static void handlePing();
static void handleResetDevice();
static void handleSetODriveState(const setOdriveStatePayload& payload);
static void handleSetJointTargets(const setJointTargetsPayload& payload);
static void handleRequestTelem();
static void handleStartHoming();
static void handleSetJointParameter(const setJointParameterPayload& payload);
static void handleEStop();
static void handleConfirmHome();

// 16-bit Float Conversion Functions
// Converts 32-bit float to 16-bit float representation for compact transmission
uint16_t float16ToUnsigned16(float value) {
  // Simple conversion: pack mantissa and exponent into 16 bits
  // This is a placeholder implementation - adjust scale/precision as needed
  if (value == 0.0f) return 0;
  
  // Scale float to fit in 16-bit unsigned (0-65535 range)
  // Adjust scale factor (1000.0f) based on your expected value range
  uint16_t result = (uint16_t)((value + 180.0f) * 181.84f);  // Map -180 to 180 degrees to 0-65535
  return result;
}

// Converts 16-bit float representation back to 32-bit float
float unsigned16ToFloat16(uint16_t bits) {
  // Reverse of float16ToUnsigned16
  float result = (float)bits / 181.84f - 180.0f;  // Map 0-65535 back to -180 to 180 degrees
  return result;
}

// Parser state machine
enum ParseState {
  WAIT_SYNC_1,
  WAIT_SYNC_2,
  READ_TYPE,
  READ_SIZE,
  READ_PAYLOAD,
  READ_CRC_1,
  READ_CRC_2
};

ParseState parseState = WAIT_SYNC_1;

// Packet currently being assembled
packet currentPacket;
uint8_t payloadIndex = 0;
uint8_t crcLo = 0;
uint8_t crcHi = 0;

// Single completed packet buffer
volatile bool packetAvailable = false;
packet completedPacket;

// Flag for ping received
volatile bool pingReceived = false;
volatile bool pingEventPending = false;
volatile bool confirmHomePending = false;
volatile bool moveToHomePending = false;

// Reset parser back to waiting for sync
void resetParser() {
  parseState = WAIT_SYNC_1;
  currentPacket = packet();
  payloadIndex = 0;
  crcLo = 0;
  crcHi = 0;
}

// Parse one incoming byte
void parseByte(uint8_t b) {
  switch (parseState) {
    case WAIT_SYNC_1:
      // Because SYNC_BYTES = 0x7FFE and Teensy is little-endian,
      // serialize() sends FE first, then 7F.
      if (b == 0x7F) {
        parseState = WAIT_SYNC_2;
      }
      break;

    case WAIT_SYNC_2:
      if (b == 0xFE) {
        currentPacket = packet();
        currentPacket.header.sync = SYNC_BYTES;
        parseState = READ_TYPE;
      } else if (b == 0x7F) {
        // Stay ready in case this byte is start of a new sync
        parseState = WAIT_SYNC_2;
      } else {
        resetParser();
      }
      break;

    case READ_TYPE:
      currentPacket.header.packetType = b;
      parseState = READ_SIZE;
      break;

    case READ_SIZE:
      currentPacket.header.payloadSize = b;

      if (b > MAX_PAYLOAD_SIZE) {
        resetParser();
      } else if (b == 0) {
        parseState = READ_CRC_1;
      } else {
        payloadIndex = 0;
        parseState = READ_PAYLOAD;
      }
      break;

    case READ_PAYLOAD:
      currentPacket.payload[payloadIndex++] = b;

      if (payloadIndex >= currentPacket.header.payloadSize) {
        parseState = READ_CRC_1;
      }
      break;

    case READ_CRC_1:
      crcLo = b;
      parseState = READ_CRC_2;
      break;

    case READ_CRC_2: {
      crcHi = b;

      uint16_t receivedCRC =
          static_cast<uint16_t>(crcLo) |
          (static_cast<uint16_t>(crcHi) << 8);
      
      uint16_t computedCRC = currentPacket.calculateCRC();
      if (receivedCRC == computedCRC) {
        currentPacket.checksum = receivedCRC;

        // Overwrite any old packet if main loop hasn't consumed it yet
        noInterrupts();
        completedPacket = currentPacket;
        packetAvailable = true;
        interrupts();
      }

      resetParser();
      break;
    }
  }
}

/*
 * Poll Serial for incoming bytes and feed parser.
 * Call this often from loop() or stateUpdate().
 */
void pollSerialPackets() {
  while (Serial.available() > 0) {
    int incoming = Serial.read();
    if (incoming >= 0) {
      parseByte(static_cast<uint8_t>(incoming));
    }
  }
}

/*
 * Returns true if a full validated packet is available.
 * Copies the packet into outPacket.
 */
static bool getNextPacket(packet& outPacket) {
  if (!packetAvailable) {
    return false;
  }

  noInterrupts();
  outPacket = completedPacket;
  packetAvailable = false;
  interrupts();

  return true;
}

/*
 * Send a packet over USB serial.
 */
// static void sendPacket(packet& pkt) {
//   std::vector<uint8_t> bytes = pkt.serialize();
static void sendPacket(const packet& pkt) {
  // serialize() updates checksum, so work on a local copy to keep input const.
  packet pktCopy = pkt;
  std::vector<uint8_t> bytes = pktCopy.serialize();
  Serial.write(bytes.data(), bytes.size());
}

/*
 * Process any available incoming packets by dispatching to handlers.
 */
void processIncomingPackets() {
  packet pkt;
  while (getNextPacket(pkt)) {
    switch (pkt.header.packetType) {
      case CMD_PING:
        handlePing();
        break;
      case CMD_RESET_DEVICE:
        handleResetDevice();
        break;
      case CMD_SET_ODRIVE_STATE:
        if (pkt.header.payloadSize == sizeof(setOdriveStatePayload)) {
          setOdriveStatePayload payload;
          memcpy(&payload, pkt.payload, sizeof(payload));
          handleSetODriveState(payload);
        }
        break;
      case CMD_SET_JOINT_TARGETS:
        if (pkt.header.payloadSize == sizeof(setJointTargetsPayload)) {
          setJointTargetsPayload payload;
          memcpy(&payload, pkt.payload, sizeof(payload));
          handleSetJointTargets(payload);
        }
        break;
      case CMD_REQUEST_TELEM:
        handleRequestTelem();
        break;
      case CMD_START_HOMING:
        handleStartHoming();
        break;
      case CMD_SET_JOINT_PARAMETER:
        if (pkt.header.payloadSize == sizeof(setJointParameterPayload)) {
          setJointParameterPayload payload;
          memcpy(&payload, pkt.payload, sizeof(payload));
          handleSetJointParameter(payload);
        }
        break;
      case CMD_ESTOP:
        handleEStop();
        break;
      case CMD_CONFIRM_HOME:
        handleConfirmHome();
        break;
      default:
        // Unknown packet type, ignore or send NACK
        sendCmdNack();
        break;
    }
  }
}

/*
 * Helper to send RESP_PONG packet.
 */
void sendCmdPong() {
  packet pong(RESP_PONG);
  sendPacket(pong);
}

/*
 * Send RESP_ACK packet.
 */
void sendCmdAck() {
  packet ack(RESP_ACK);
  sendPacket(ack);
}

/*
 * Send RESP_NACK packet.
 */
void sendCmdNack() {
  packet nack(RESP_NACK);
  sendPacket(nack);
}

/*
 * Send TELEM_JOINT_DATA packet.
 */
void sendTelemJointData(const telemJointDataPayload& payload) {
  packet pkt(TELEM_JOINT_DATA, payload);
  sendPacket(pkt);
}

/*
 * Send TELEM_STATUS packet.
 */
void sendTelemStatus(const telemStatusPayload& payload) {
  packet pkt(TELEM_STATUS, payload);
  sendPacket(pkt);
}

/*
 * Send LOG_MESSAGE packet.
 */
void sendLogMessage(const char* message) {
  packet pkt(LOG_MESSAGE);
  size_t len = strlen(message);
  if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;
  memcpy(pkt.payload, message, len);
  pkt.header.payloadSize = len;
  pkt.checksum = pkt.calculateCRC();
  sendPacket(pkt);
}

/*
 * Send ERROR_MESSAGE packet.
 */
void sendErrorMessage(const char* message) {
  packet pkt(ERROR_MESSAGE);
  size_t len = strlen(message);
  if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;
  memcpy(pkt.payload, message, len);
  pkt.header.payloadSize = len;
  pkt.checksum = pkt.calculateCRC();
  sendPacket(pkt);
}

void buildTelemJointPayload(telemJointDataPayload& payload, int jointID, float angleDeg, float velocityDegPerSec) {
	payload.joints[jointID].jnAngle = float16ToUnsigned16(angleDeg);
	payload.joints[jointID].jnVelocity = float16ToUnsigned16(velocityDegPerSec);
}

packet::packet() {
	header.sync = SYNC_BYTES;
	header.packetType = 0;
	header.payloadSize = 0;
	checksum = 0;
	memset(payload, 0, MAX_PAYLOAD_SIZE);
}

packet::packet(uint8_t type) {
	header.sync = SYNC_BYTES;
	header.packetType = type;
	header.payloadSize = 0;
	checksum = calculateCRC();
}

std::vector<uint8_t> packet::serialize() {
	//Teensy will send little-endian
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

uint16_t packet::calculateCRC() {
	uint16_t crc = 0x0000;
	crc = updateCRC(crc, header.packetType);
	crc = updateCRC(crc, header.payloadSize);
	for (uint8_t i = 0; i < header.payloadSize; ++i) {
		crc = updateCRC(crc, payload[i]);
	}
	return crc;
}

uint16_t packet::updateCRC(uint16_t crc, uint8_t data) {
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

// Handler implementations

static void handlePing() {
  pingReceived = true;  // Mark that host has connected via ping
  pingEventPending = true;
  sendCmdPong();
}

static void handleResetDevice() {
  // TODO: Implement device reset, perhaps restart Teensy or reset state
  // For now, just send ACK
  sendCmdAck();
}

static void handleSetODriveState(const setOdriveStatePayload& payload) {
  // TODO: Set ODrive state for joints specified by jointMask
  // For now, send ACK
  sendCmdAck();
}

static void handleSetJointTargets(const setJointTargetsPayload& payload) {
  // TODO: Set joint targets (velocity and torque)
  // For now, send ACK
  sendCmdAck();
}

static void handleRequestTelem() {
  // TODO: Send current telemetry data
  // For now, send ACK
  sendCmdAck();
}

static void handleStartHoming() {
  moveToHomePending = true;
  sendCmdAck();
}

static void handleSetJointParameter(const setJointParameterPayload& payload) {
  // TODO: Set joint parameter
  // For now, send ACK
  sendCmdAck();
}

static void handleEStop() {
  // TODO: Emergency stop
  // For now, send ACK
  sendCmdAck();
}

static void handleConfirmHome() {
  confirmHomePending = true;
  sendCmdAck();
}