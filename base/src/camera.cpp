// Captures synchronized frames from both OV9281 global-shutter cameras through libcamera, with
// locked exposure/gain and a short exposure so the IR LED is a small, sharp blob. Reports
// camera disconnects, frame timeouts and dropped frames to faults.
//
// For: Base (Raspberry Pi 5)

#include "camera.h"
