#include "LEDs.h"

#define FALSE 0
#define TRUE 1

namespace Led {
    /* Receiving satisfactory power for Teensy operation */
    led_t powerLED = {7, POWER};

    /* YELLOW: Receiving data from encoders/ODrives, transitively ready to send data to PC */
    led_t dataLED = {8, DATA};

    /* RED: Toggle errorLED on an error mode */
    led_t errorLED = {28, ERROR};

    void setup() {
        initLED(&Led::powerLED);
        initLED(&Led::dataLED);
        initLED(&Led::errorLED);
    }
}
// intialize the LEDs, set pin modes, and set initial states
void initLED(led_t* l) {
    pinMode(l->pin, OUTPUT);
}
// turn off led
void ONToggleLED(led_t* l) {
    digitalWrite(l->pin, HIGH);
    delay(5);
}
// turn on led
void OFFToggleLED(led_t* l) {
    digitalWrite(l->pin, LOW);
}