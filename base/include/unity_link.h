// Bluetooth Classic serial (SPP/RFCOMM) link to the Unity laptop; on Windows the Pi shows up as a
// COM port. Sends the newest handle position each cycle and receives commands from Unity.
//
// Protocol (simple text lines, '\n' terminated):
//   Pi -> Unity:  "POS x y z"      handle position
//   Unity -> Pi:  "CMD <id>"       command with an integer id
//   Pi -> Unity:  "ERR <message>"  error report (optional)
// The V1-style binary framing (serial_framing.h) is NOT used by this link yet; switch to it later
// if CRC protection is needed. Fallback if Bluetooth is too slow: the laptop joins the Pi's
// network with a USB WiFi adapter and this link runs over UDP instead.
//
// Pi setup: `sudo rfcomm watch hci0 1` must be running so /dev/rfcomm0 appears once Unity
// connects (see SETUP.md section 5). Unity side: unity/PiBluetoothLinkSimple.cs.
//
// For: Base (Raspberry Pi 5)

#ifndef UNITY_LINK_H
#define UNITY_LINK_H

// Open /dev/rfcomm0 (non-blocking, raw). Returns false if it does not exist yet.
bool unityLinkBegin();

// Wait until /dev/rfcomm0 can be opened (the laptop has connected), or time out.
bool unityLinkWaitForConnection(int timeoutMs = 30000);

// Send the newest handle position. Returns false if not connected or the write failed.
// TODO: take the pose type once pose.h defines it.
bool unityLinkSendPose(float x, float y, float z);

// Send "ERR <message>" to Unity.
bool unityLinkSendError(const char* message);

// Read incoming bytes, split into lines, and dispatch complete commands. Call every cycle.
void unityLinkPoll();

// Whether the laptop is currently connected.
bool unityLinkIsConnected();

// Register a callback that runs when a "CMD <id>" line arrives.
void unityLinkOnCommand(void (*callback)(int id));

// Register a callback that runs when the laptop disconnects.
void unityLinkOnDisconnect(void (*callback)());

#endif
