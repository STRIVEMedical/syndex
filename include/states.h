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

//Homing state functions
void startHoming();
bool verifyHoming();

//Ready state functions
void enableI2CPacketSend();
void enableODrivePacketSend();

//PowerOff state functions
void powerOffPeripherals();

//Error Enumeration (Different types of errors that may occur)
//ENCODER_ERROR state?

//Error state functions
bool errorCheck();
void sendStateErrorLog();
void stopODrives();
void turnOnErrorLED();
void errorRecovery();

#endif