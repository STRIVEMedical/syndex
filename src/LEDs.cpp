#include "LEDs.h"

namespace Led {
    led_t powerLED = {15, POWER};   
    led_t dataLED = {14, DATA};  
    led_t errorLED = {13, ERROR};   

    void setup() {
        initLED(&powerLED);
        initLED(&dataLED);
        initLED(&errorLED);
    }

    void loop() {

        led_t* leds[] = {&powerLED,  &dataLED,  &errorLED};
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