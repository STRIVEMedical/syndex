#include "LEDs.h"

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