// UDP link from the handle to the Pi base's WiFi access point: sends orientation, gyro/accel,
// trigger and dock state with sequence numbers, and receives vibration commands. Stops the
// vibration motor on link loss.
//
// For: Handle ESP32-S3

#include <Arduino.h>

#include "base_link.h"
#include "messages.h"
#include "errors.h"
