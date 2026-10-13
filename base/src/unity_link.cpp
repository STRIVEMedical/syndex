// Bluetooth Classic serial (SPP/RFCOMM) link to the Unity laptop; on Windows the Pi shows up as a
// COM port. Sends the newest handle pose each cycle and receives vibration/tool/state commands
// using V1-style serial framing (see serial_framing.h). Fallback if Bluetooth is too slow: the
// laptop joins the Pi's network with a USB WiFi adapter and this link runs over UDP instead.
//
// For: Base (Raspberry Pi 5)

#include "unity_link.h"
#include "messages.h"
#include "serial_framing.h"

bool unityLinkBegin() {
    // TODO: open the Bluetooth serial (SPP) port and register the service.
    return false;
}

bool unityLinkWaitForConnection() {
    // TODO: block until the laptop connects (or time out).
    return false;
}

bool unityLinkSendPose() {
    // TODO: send the newest handle pose (take the pose type once pose.h defines it).
    return false;
}

void unityLinkPoll() {
    // TODO: read incoming bytes, feed the serial-framing parser, and dispatch complete commands.
}

bool unityLinkIsConnected() {
    // TODO: report whether the laptop is currently connected.
    return false;
}

void unityLinkOnDisconnect(void (*callback)()) {
    // TODO: register a callback that runs when the laptop disconnects.
    (void)callback;
}
