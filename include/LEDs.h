#pragma once

#include <Arduino.h>

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

namespace LED {
    /* Receiving satisfactory power for Teensy operation */
    extern led_t powerLED;

    /* Receiving data from encoders/ODrives, transitively ready to send data to PC */
    extern led_t dataLED;

    /* Toggle errorLED on an error mode */
    extern led_t errorLED;

    void setup();

}

void initLED(led_t* l);

void ONToggleLED(led_t* l);

void OFFToggleLED(led_t* l);