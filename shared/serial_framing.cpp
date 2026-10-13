// V1-style serial frame format and parser for the Bluetooth Unity link (port from V1 USB.cpp on
// the V1 branch). Frames are 0x7F 0xFE, type, length, payload, CRC-16/ARC; encoding uses
// packet::serialize() in messages.h, this file is the receive-side byte-stream parser.
//
// For: Base (Raspberry Pi 5). Lives in shared/ but only the base build compiles it.

#include "serial_framing.h"

void serialFramingReset() {
    // TODO: reset the parser to wait for the sync bytes.
}

bool serialFramingFeedByte(uint8_t byte) {
    // TODO: advance the parser by one received byte; return true once a complete, CRC-valid packet is ready.
    (void)byte;
    return false;
}

bool serialFramingGetPacket(packet& out) {
    // TODO: copy out the completed packet; return false if none is ready.
    (void)out;
    return false;
}
