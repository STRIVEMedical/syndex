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
  // Initialize main Serial (USB) for Unity communication
  MAIN_SERIAL.begin(115200);
  delay(100);
  
  // Initialize debug Serial (UART 1) for PuTTY monitoring
  DEBUG_SERIAL.begin(115200);
  delay(100);

  // Joint map/state must exist before READY telemetry path runs.
  initJoints();

  // Pre-register ODrive callbacks before CAN starts (prevents "missing callback" error)
  preInitOdriveCallbacks();

  DEBUG_SERIAL.println("\n========== SYSTEM STARTUP ==========");
  DEBUG_SERIAL.println("State-machine test harness started");
  DEBUG_SERIAL.println("Debug output: Serial1 (PuTTY)");
  DEBUG_SERIAL.println("Main comms: Serial (USB/Unity)");
  DEBUG_SERIAL.println("====================================\n");
}


void loop() {
  // Keep CAN traffic serviced continuously (heartbeats/status/callbacks).
  pumpEvents(can_intf);

  state_e prevState = currState;
  stateUpdate();

  // Log state transitions to debug serial (PuTTY)
  if (currState != prevState) {
    DEBUG_SERIAL.print("STATE: ");
    DEBUG_SERIAL.print(stateName(prevState));
    DEBUG_SERIAL.print(" -> ");
    DEBUG_SERIAL.println(stateName(currState));
  }

  // DEBUG: Print encoder readings every 500ms for testing
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    readJointAngles();      // Update all joint angle/velocity values
    printJointStatus();     // Print all joints in easy-to-read format
  }

  // DEBUG: Dump ODrive config every 3 seconds for testing
  static unsigned long lastConfigDump = 0;
  if (millis() - lastConfigDump > 3000) {
    lastConfigDump = millis();
    dumpODriveConfig();     // Print ODrive hardware status and config
  }
}