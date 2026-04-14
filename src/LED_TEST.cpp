#include <Arduino.h>
#include "LEDs.h"

void setup() {
    Serial.begin(9600);

    // Wait max 3 seconds for Serial — don't block forever
    // Allows Teensy to run standalone without a PC connected

    uint32_t startTime = millis();

    while (!Serial && (millis() - startTime < 3000)) { ; }

    Led::setup();
    
}

void loop() {
    Led::loop();
}