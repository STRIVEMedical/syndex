#include "states.h"

#include <Arduino.h>
#include "comms.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"
#include "main.h"
#include "states.h"
#include "admittance_controller.h"
#include <TeensyID.h>

#include "USB.h"

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
    // Initialize admittance model parameters once at boot.
    initAdmittanceController();
    if (!initCommunications()) {
      SerialUSB1.println("Communication init failed");
      while (true) {
        delay(1000);
      }
    }

    // Put ODrive 2 into closed-loop velocity control for a slow spin test.
    // enable_closed_loop(odrv2, odrv2_user_data);
    // odrv2.setControllerMode(
    //     ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
    //     ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    // odrv2.setVelocity(0.5f);
    // SerialUSB1.println("Started slow velocity test on ODrive 2");



    SerialUSB1.print("[SYS] USB serial: ");
    SerialUSB1.println(teensyUsbSN());
    SerialUSB1.println("[SYS] Startup complete");


}

static unsigned long lastStatusPrintMs = 100;


void loop() {

  pumpEvents(can_intf);
  // Poll for incoming USB packets and process them
  pollSerialPackets();
  processIncomingPackets();

  state_e prevState = currState;
  stateUpdate();

  // Log state transitions to debug serial (PuTTY)
  if (currState != prevState) {
    SerialUSB1.print("STATE: ");
    SerialUSB1.print(stateName(prevState));
    SerialUSB1.print(" -> ");
    SerialUSB1.println(stateName(currState));
  }
  
  //   readJointAngles();
  // if (millis() - lastStatusPrintMs >= 1000) {
  //     lastStatusPrintMs = millis();
      
  //     printJointStatus();
  // }



}