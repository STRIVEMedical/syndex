#include <Arduino.h>
#include "comms.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"
#include "main.h"
#include "states.h"
#include <TeensyID.h>

#include "USB.h"

static unsigned long lastPrint = 0;


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
    // Initialize BOTH USB ports first, before anything else
    Serial.begin(115200);
    SerialUSB1.begin(115200);

    // Give USB stack time to fully enumerate BOTH ports
    // 2000ms is conservative but reliable for dual serial
    delay(2000);

    // NOW it's safe to init joints, CAN, etc.
    initJoints();
    preInitOdriveCallbacks();

    SerialUSB1.printf("USB Serial: %u\n", teensyUsbSN());
    SerialUSB1.println("Startup complete");
}


void loop() {
  // Keep CAN traffic serviced continuously (heartbeats/status/callbacks).
  pumpEvents(can_intf);

  // Poll for incoming USB packets and process them
  pollSerialPackets();
  processIncomingPackets();

  state_e prevState = currState;
  // stateUpdate();

  // Log state transitions to debug serial (PuTTY)
  // if (currState != prevState) {
  //   SerialUSB1.print("STATE: ");
  //   SerialUSB1.print(stateName(prevState));
  //   SerialUSB1.print(" -> ");
  //   SerialUSB1.println(stateName(currState));
  // }

  // DEBUG: Print encoder readings every 500ms for testing
  // static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    // readJointAngles();      // Update all joint angle/velocity values
    // printJointStatus();     // Print all joints in easy-to-read format
    printOdrvIQcurrents();
  }

  // DEBUG: Dump ODrive config every 3 seconds for testing
  // static unsigned long lastConfigDump = 0;
  // if (millis() - lastConfigDump > 3000) {
  //   lastConfigDump = millis();
  //   dumpODriveConfig();     // Print ODrive hardware status and config
  // }
}