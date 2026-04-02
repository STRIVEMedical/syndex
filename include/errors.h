#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdbool.h>

typedef enum {
    NO_ERROR,
    ODRIVE_ERROR,
    I2C_ERROR,
    BUTTON_ERROR,
    LED_ERROR,
    CONNECTION_ERROR,
} errorCode_e;

extern errorCode_e currError;

//Reports if an error has occured
void setError(errorCode_e err);
//Checks current error
errorCode_e getError();
//Resets the state after recovery
void clearError(void);
//Checks whether the error still exists
bool errorCheck(void);
#endif