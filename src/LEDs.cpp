#include "LEDs.h"

namespace Led {
    led_t powerLed = {15, POWER};   // Green
    led_t dataLed = {14, DATA};    // Yellow
    led_t errorLed = {13, ERROR};   // Red

    void setup() {
        initLED(&powerLed);
        initLED(&dataLed);
        initLED(&errorLed);
    }

    void loop() {

        led_t* leds[] = { &powerLed,  &dataLed,  &errorLed  };
        const char* labels[] = {"Power LED", "Data LED", "Error LED" };
        const int count = sizeof(leds) / sizeof(leds[0]);

        for (int i = 0; i < count; i++) {
            Serial.print(labels[i]);
            //Serial.println(" ON");

            ONToggleLED(leds[i]);
            //delay(500);

            //OFFToggleLED(leds[i]);
            //delay(100);
        }
    }
}

void initLED(led_t* l) {
    pinMode(l->pin, OUTPUT);
    digitalWrite(l->pin, LOW);
}

void ONToggleLED(led_t* l) {
    digitalWrite(l->pin, HIGH);
}

void OFFToggleLED(led_t* l) {
    digitalWrite(l->pin, LOW);
}