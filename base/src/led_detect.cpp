// Finds the 850 nm IR LED blob in each camera image (threshold, contour, sub-pixel centroid)
// and rejects reflections and ambient-IR false detections. Reports when the LED is not visible
// in one or both cameras.
//
// For: Base (Raspberry Pi 5)

#include "led_detect.h"
