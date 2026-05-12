#ifndef STATES_H
#define STATES_H

#include <stdbool.h>
#include "errors.h"
typedef enum {
    BOOTUP,
    IDLE,
    CONNECTED,
    HOMING,
    READY,
    POWERINGOFF,
    ERROR_STATE,
} state_e;

//Current device state
extern state_e currState;

// Host connection flag: set to true when ping is received from host
extern volatile bool pingReceived;

void stateUpdate();

// Queues CMD_START_HOMING using the current firmware control mode to decide
// whether admittance should be restored after the homing move.
void requestStartHoming();

//BootUp state functions
bool verifyODrive();
bool verifyI2C();
bool verifyLED();

//Idle state functions
bool pollCmdPing();

//Connected state functions
bool verifyUSB();
bool verifyODriveComms();
bool allConnectionsReady();
bool isHomed();


//Ready state functions
void enableI2CPacketSend();
void enableODrivePacketSend();

//PowerOff state functions
void powerOffPeripherals();

//Error state functions
bool errorCheck();
void sendStateErrorLog();
void stopODrives();
void turnOnErrorLED();
void errorRecovery();

#endif
