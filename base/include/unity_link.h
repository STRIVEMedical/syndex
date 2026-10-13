// Bluetooth Classic serial (SPP/RFCOMM) link to the Unity laptop; on Windows the Pi shows up as a
// COM port. Sends the newest handle pose each cycle and receives vibration/tool/state commands
// using V1-style serial framing (see serial_framing.h). Fallback if Bluetooth is too slow: the
// laptop joins the Pi's network with a USB WiFi adapter and this link runs over UDP instead.
//
// For: Base (Raspberry Pi 5)

#ifndef UNITY_LINK_H
#define UNITY_LINK_H

// TODO: open the Bluetooth serial (SPP) port and register the service.
bool unityLinkBegin();

// TODO: block until the laptop connects (or time out).
bool unityLinkWaitForConnection();

// TODO: send the newest handle pose (take the pose type once pose.h defines it).
bool unityLinkSendPose();

// TODO: read incoming bytes, feed the serial-framing parser, and dispatch complete commands.
void unityLinkPoll();

// TODO: report whether the laptop is currently connected.
bool unityLinkIsConnected();

// TODO: register a callback that runs when the laptop disconnects.
void unityLinkOnDisconnect(void (*callback)());

#endif
