#pragma once

#include <Arduino.h>
#include "states.h"

typedef enum {
    POWER,
    DATA,
    ERROR,
} ledType_e;

typedef struct {
    uint8_t pin; // GPIO Pin #
    ledType_e type; // LEDs will be OUTPUT
    /* 
    uint8_t ledState; // Current button state
    uint8_t lastLedState; // Last state from button, active LOW
    */
} led_t;

namespace Led {
    /* Receiving satisfactory power for Teensy operation */
    extern led_t powerLed;

    /* Receiving data from encoders/ODrives, transitively ready to send data to PC */
    extern led_t dataLed;

    /* Toggle errorLED on an error mode */
    extern led_t errorLed;

    void setup();

}

void initLED(led_t* l);

void ONToggleLED(led_t* l);

void OFFToggleLED(led_t* l);

#ifdef DEBUG_MODE
void setDebugLEDs(state_e state);
#endif