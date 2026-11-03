// UDP link to the two handles: receives orientation, gyro/accel, trigger and dock state,
// routes it by handle ID, and discards stale or out-of-order packets. Also forwards vibration
// commands back to the correct handle.
//
// For: Base (Raspberry Pi 5)

// UDP link to the two handles: receives orientation, gyro/accel, trigger and dock state,
// routes it by handle ID, and discards stale or out-of-order packets. Also forwards vibration
// commands back to the correct handle.
//
// For: Base (Raspberry Pi 5)

#ifndef HANDLE_LINK_H
#define HANDLE_LINK_H

#include <stdint.h>
#include "messages.h"
bool handle_link_send_ir(uint8_t handleId, bool enabled, uint8_t brightness);
bool handle_link_init();
void handle_link_poll(int timeout_ms);                                 // call repeatedly from the main loop
bool handle_link_get_state(uint8_t handleId, handleStatePayload &out); // true only if new since last call
bool handle_link_send_vibrate(uint8_t handleId, uint8_t intensity, uint16_t durationMs);
bool handle_link_is_up(uint8_t handleId, int timeout_ms);
void handle_link_close();

#endif // HANDLE_LINK_H
