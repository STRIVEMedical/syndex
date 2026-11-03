// UDP link from the handle to the Pi base's WiFi access point: sends orientation, gyro/accel,
// trigger and dock state with sequence numbers, and receives vibration commands. Stops the
// vibration motor on link loss.
//
// For: Handle ESP32-S3

// UDP link from the handle to the Pi base's WiFi access point: sends orientation, gyro/accel,
// trigger and dock state with sequence numbers, and receives vibration commands. Stops the
// vibration motor on link loss.
//
// For: Handle ESP32-S3

#ifndef BASE_LINK_H
#define BASE_LINK_H

#include <Arduino.h>
#include "messages.h"

bool base_link_init();
void base_link_update();                            // call every loop(), never blocks
bool base_link_send_state(handleStatePayload s);    // fills meta (handle ID + seq) for you
bool base_link_get_vibrate(vibrateCmdPayload &out); // true only if a new command arrived
bool base_link_get_ir(irLedCmdPayload &out);        // true only if a new command arrived
bool base_link_is_up();

#endif // BASE_LINK_H