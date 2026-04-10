#include "buttons.h"

#define TRIGGER_MIN_ADC 0
#define TRIGGER_MAX_ADC 4046

namespace buttonPins {
    button_t powerButton = {37, INPUT, HIGH, HIGH};
    button_t toolSelect  = {39, INPUT, HIGH, HIGH};
    button_t triggerInput = {40, INPUT, HIGH, HIGH};
}

void buttonInit(button_t* b) {
    pinMode(b->pin, b->io);
    b->buttonState = digitalRead(b->pin);
    b->lastButtonState = b->buttonState;
}

void buttonUpdate(button_t* b) {
    b->lastButtonState = b->buttonState;
    b->buttonState = digitalRead(b->pin);
}

void Buttons::setup() {
    analogReadResolution(12); // trigger reports values in 0-4095 range
    buttonInit(&buttonPins::powerButton);
    buttonInit(&buttonPins::triggerInput);
    buttonInit(&buttonPins::toolSelect);
};

void ToolCycleButton(button_t* b) {
    static int lastReading = HIGH;
    static int stableState = HIGH;
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;

    int reading = digitalRead(b->pin);

    if (reading != lastReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != stableState) {
            stableState = reading;

            if (stableState == LOW) {
                Serial.println("Cycle button Pressed");
            } else {
                Serial.println("Cycle button Released");
            }
        }
    }

    lastReading = reading;
}

void testPowerButton(button_t* b) {
    static int lastReading = HIGH;
    static int stableState = HIGH;
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 5;

    int reading = digitalRead(b->pin);

    if (reading != lastReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != stableState) {
            stableState = reading;

            if (stableState == LOW) {
                Serial.println("Power button Released");
            } else {
                Serial.println("Power button Pressed");
            }
        }
    }

    lastReading = reading;
}

/* ========== TRIGGER CODE ========== */

float getTriggerDepth(button_t* b) {
    int rawValue = constrain(analogRead(b->pin), TRIGGER_MIN_ADC, TRIGGER_MAX_ADC);
    return (float)(rawValue - TRIGGER_MIN_ADC) / (TRIGGER_MAX_ADC - TRIGGER_MIN_ADC);
}

void triggerPulled(button_t* b) {

    static bool wasPressed = false;

    float depth = getTriggerDepth(b);

    if (depth > 0.01f) {
        if (!wasPressed) {
            //Serial.print("Trigger depth: ");
            //Serial.println(depth);
            Serial.println("Trigger Pressed");
            wasPressed = true;
        }
    } else {
        if (wasPressed) {
            Serial.println("Trigger Released");
            wasPressed = false;
        }
    }
}