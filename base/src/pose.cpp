// Combines the triangulated LED XYZ with the handle's IMU orientation to produce each handle's
// pose: position, orientation, tracking validity and timestamp. This is the single source of
// truth for telemetry.
//
// For: Base (Raspberry Pi 5)

#include "pose.h"
