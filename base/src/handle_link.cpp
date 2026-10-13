// UDP link to the two handles: receives orientation, gyro/accel, trigger and dock state,
// routes it by handle ID, and discards stale or out-of-order packets. Also forwards vibration
// commands back to the correct handle.
//
// For: Base (Raspberry Pi 5)

#include "handle_link.h"
#include "messages.h"
