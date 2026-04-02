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

  // Pre-register ODrive callbacks before CAN starts (prevents "missing callback" error)
  preInitOdriveCallbacks();

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

  // DEBUG: Print encoder readings every 500ms for testing
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    readJointAngles();      // Update all joint angle/velocity values
    printJointStatus();     // Print all joints in easy-to-read format
  }
}