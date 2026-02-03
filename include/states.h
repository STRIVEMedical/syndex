#ifndef STATES_H
#define STATES_H

#include <stdbool.h>

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

void stateUpdate();

//BootUp state functions
bool verifyODrive();
bool verifyI2C();
bool verifyButton();
bool verifyLED();

//Idle state functions
bool pollCmdPing();
void sendCmdPong();

//Connected state functions
bool verifyUSB();
bool verifyODriveComms();
bool verifyConnectedI2CDevices();
bool allConnectionsReady();
bool isHomed();

//Homing state functions
void startHoming();
bool verifyHoming();

//Ready state functions
void enableI2CPacketSend();
void enableODrivePacketSend();

//PowerOff state functions
void powerOffODrives();
void powerOffPeripherals();

//Error Enumeration (Different types of errors that may occur)
//ENCODER_ERROR state?
typedef enum {
    ODRIVE_ERROR,
    I2C_ERROR,
    BUTTON_ERROR,
    LED_ERROR,
    CONNECTION_ERROR,
    NO_ERROR,
} errorCode_e;

extern errorCode_e currError;

//Reports if an error has occured
void setError(errorCode_e);
//Checks current error
errorCode_e getError();
//Resets the state after recovery
void clearError(void);

//Error state functions
bool errorCheck();
void sendErrMessage();
void endPower();
void stopODrives();
void turnOnErrorLED();
void errorRecovery();

#endif