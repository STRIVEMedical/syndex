#include <stdint.h>
#include <vector>
#include <cstring>
#include "USB.h"

void buildTelemJointPayload(telemJointDataPayload& payload, int jointID, float angleDeg, float velocityDegPerSec) {
	payload.joints[jointID].jnAngle = (uint16_t)angleDeg;
	payload.joints[jointID].jnVelocity = (uint16_t)velocityDegPerSec;
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
	std::vector<uint8_t> buffer;
	uint8_t* headerPtr = reinterpret_cast<uint8_t*>(&header);
	buffer.insert(buffer.end(), headerPtr, headerPtr + sizeof(packetHeader));
	buffer.insert(buffer.end(), payload, payload + header.payloadSize);

	checksum = calculateCRC();
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