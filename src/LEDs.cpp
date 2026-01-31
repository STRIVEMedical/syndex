#include "LEDs.h"

#define FALSE 0
#define TRUE 1

namespace LED {
    /* Receiving satisfactory power for Teensy operation */
    led_t powerLED = {7, POWER};

    /* Receiving data from encoders/ODrives, transitively ready to send data to PC */
    led_t dataLED = {8, DATA};

    /* Toggle errorLED on an error mode */
    led_t errorLED = {28, ERROR};

    void setup() {
        initLED(&LED::powerLED);
        initLED(&LED::dataLED);
        initLED(&LED::errorLED);
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