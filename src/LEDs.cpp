#include "LEDs.h"
#include "states.h"
#include "DEBUG.h"

#define FALSE 0
#define TRUE 1

namespace Led {
    /* Receiving satisfactory power for Teensy operation */
    led_t powerLed = {7, POWER};
    led_t dataLed;
    /* YELLOW: Receiving data from encoders/ODrives, transitively ready to send data to PC */
    led_t dataLED = {8, DATA};

    /* RED: Toggle errorLED on an error mode */
    led_t errorLed = {28, ERROR};

    void setup() {
        initLED(&Led::powerLed);
        initLED(&Led::dataLed);
        initLED(&Led::errorLed);
    }
}

void initLED(led_t* l) {
    pinMode(l->pin, OUTPUT);
}

void ONToggleLED(led_t* l) {
    digitalWrite(l->pin, HIGH);
    delay(5);
}

void OFFToggleLED(led_t* l) {
    digitalWrite(l->pin, LOW);
}

#ifdef DEBUG_MODE
void setDebugLEDs(state_e state) {
    // Turn off all LEDs first
    OFFToggleLED(&Led::powerLed);
    OFFToggleLED(&Led::dataLed);
    OFFToggleLED(&Led::errorLed);

    switch (state) {
        case BOOTUP:
            // All off - system initializing
            break;
        case IDLE:
            // Power LED on - waiting for connection
            ONToggleLED(&Led::powerLed);
            break;
        case CONNECTED:
            // Power and Data on - connected, checking systems
            ONToggleLED(&Led::powerLed);
            ONToggleLED(&Led::dataLed);
            break;
        case HOMING:
            // Power and Error on - homing in progress
            ONToggleLED(&Led::powerLed);
            ONToggleLED(&Led::errorLed);
            break;
        case READY:
            // All on - system ready
            ONToggleLED(&Led::powerLed);
            ONToggleLED(&Led::dataLed);
            ONToggleLED(&Led::errorLed);
            break;
        case POWERINGOFF:
            // Data and Error on - shutting down
            ONToggleLED(&Led::dataLed);
            ONToggleLED(&Led::errorLed);
            break;
        case ERROR_STATE:
            // Error LED on - error occurred
            ONToggleLED(&Led::errorLed);
            break;
        default:
            // All blinking or something - unknown state
            break;
    }
}
#endif // DEBUG_MODE