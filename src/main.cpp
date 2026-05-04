#include "states.h"

#include <Arduino.h>
#include "comms.h"
#include "odrive.h"
#include "LEDs.h"
#include "joint.h"
#include "main.h"
#include "states.h"
#include "admittance_controller.h"
#include <TeensyID.h>
#include "USB.h"

void setup() {
    // Initialize BOTH USB ports first, before anything else
    Serial.begin(115200);
    SerialUSB1.begin(115200);

    // Guarantee LEDs configure
    Led::setup();

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

    SerialUSB1.print("[SYS] USB serial: ");
    SerialUSB1.println(teensyUsbSN());
    SerialUSB1.println("[SYS] Startup complete");

}


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
  
}