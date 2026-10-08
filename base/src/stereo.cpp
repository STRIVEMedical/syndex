// Loads the saved stereo calibration and triangulates the LED's XYZ position, in the console
// coordinate frame, from its pixel coordinates in the two cameras. Reports an out-of-volume
// state instead of clamping.
//
// For: Base (Raspberry Pi 5)

#include "stereo.h"
