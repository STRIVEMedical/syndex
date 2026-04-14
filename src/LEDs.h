#pragma once

#include <Arduino.h>
#include "states.h"

typedef enum {
    POWER,
    DATA,
    ERROR,
} ledType_e;

typedef struct {
    uint8_t pin;
    ledType_e type;
} led_t;

namespace Led {
    extern led_t powerLed;   // Green:  pin 15 - satisfactory power for Teensy
    extern led_t dataLed;    // Yellow: pin 14 - receiving data from encoders/ODrives
    extern led_t errorLed;   // Red:    pin 13 - toggled on error state

    void setup();
    void loop();
}

void initLED(led_t* l);
void ONToggleLED(led_t* l);
void OFFToggleLED(led_t* l);