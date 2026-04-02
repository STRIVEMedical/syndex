#include <Arduino.h>
#include "comms.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"
#include "main.h"
#include "states.h"

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

void setup() {
  Serial.begin(115200);
  delay(200);

  // Joint map/state must exist before READY telemetry path runs.
  initJoints();

  Serial.println("State-machine test harness started");
  Serial.println("Waiting for CMD_PING to enter CONNECTED/READY telemetry flow...");
}


void loop() {
  // Keep CAN traffic serviced continuously (heartbeats/status/callbacks).
  pumpEvents(can_intf);

  state_e prevState = currState;
  stateUpdate();

  // Log transitions to verify state-machine behavior quickly over USB serial.
  if (currState != prevState) {
    Serial.print("STATE: ");
    Serial.print(stateName(prevState));
    Serial.print(" -> ");
    Serial.println(stateName(currState));
  }
}