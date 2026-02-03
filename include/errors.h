#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdbool.h>

typedef enum {
    NO_ERROR,
    ODRIVE_ERROR,
    I2C_ERROR,
    BUTTON_ERROR,
    LED_ERROR,
} errorCode_e;

//Reports if an error has occured
void setError(errorCode_e);
//Checks current error
void getError(void);
//Resets the state after recovery
void clearError(void);
//Checks whether the error still exists
bool hasError(void);
#endif