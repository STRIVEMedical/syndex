#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "comms.h"
#include "LEDs.h"
#include "joint.h"


extern Joint joints[NUM_JOINTS];

static const char* stateName(state_e state) {
  switch (state) {
    case BOOTUP: return "BOOTUP";
    case IDLE: return "IDLE";
    case CONNECTED: return "CONNECTED";
    case HOMING: return "HOMING";
    case READY: return "READY";
    case POWERINGOFF: return "POWERINGOFF";
    case ERROR_STATE: return "ERROR_STATE";
    default: return "UNKNOWN";
  }
}

void setup();

void loop();


#endif // MAIN_H
