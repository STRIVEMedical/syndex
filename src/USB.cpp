#include <stdint.h>
#include <vector>
#include <cstring>
#include <cmath>
#include "USB.h"

uint16_t float16ToUnsigned16(float value) {
	if (std::isnan(value)) {
		return 0x7E00;
	}

	union {
		float f;
		uint32_t u;
	} in = {value};

	uint32_t sign = (in.u >> 16) & 0x8000;
	int32_t exp = ((in.u >> 23) & 0xFF) - 127 + 15;
	uint32_t mantissa = in.u & 0x7FFFFF;

	if (exp <= 0) {
		if (exp < -10) {
			return static_cast<uint16_t>(sign);
		}
		mantissa |= 0x800000;
		uint32_t shifted = mantissa >> (1 - exp + 13);
		if ((mantissa >> (1 - exp + 12)) & 0x1) {
			shifted++;
		}
		return static_cast<uint16_t>(sign | shifted);
	}

	if (exp >= 31) {
		return static_cast<uint16_t>(sign | 0x7C00);
	}

	uint16_t half = static_cast<uint16_t>(sign | (exp << 10) | (mantissa >> 13));
	if (mantissa & 0x1000) {
		half++;
	}
	return half;
}

float unsigned16ToFloat16(uint16_t bits) {
	uint32_t sign = (static_cast<uint32_t>(bits & 0x8000)) << 16;
	uint32_t exp = (bits >> 10) & 0x1F;
	uint32_t mantissa = bits & 0x3FF;
	uint32_t out;

	if (exp == 0) {
		if (mantissa == 0) {
			out = sign;
		} else {
			exp = 1;
			while ((mantissa & 0x400) == 0) {
				mantissa <<= 1;
				exp--;
			}
			mantissa &= 0x3FF;
			out = sign | ((exp + (127 - 15)) << 23) | (mantissa << 13);
		}
	} else if (exp == 31) {
		out = sign | 0x7F800000 | (mantissa << 13);
	} else {
		out = sign | ((exp + (127 - 15)) << 23) | (mantissa << 13);
	}

	union {
		uint32_t u;
		float f;
	} result = {out};

	return result.f;
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