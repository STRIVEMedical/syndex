#include "buttons.h"

#define TRIGGER_MIN_ADC 0
#define TRIGGER_MAX_ADC 4046

namespace buttonPins {
    button_t powerButton = {37, INPUT, HIGH, HIGH};
    button_t toolSelect  = {39, INPUT, HIGH, HIGH};
    button_t triggerInput = {40, INPUT, HIGH, HIGH};
}

static int pollEdge(button_t* b, DebouncedButton& db) {
    int reading = digitalRead(b->pin);
    unsigned long now = millis();
    if (reading != db.lastReading) {
        db.lastDebounceTime = now;
    }
    db.lastReading = reading;
    if ((now - db.lastDebounceTime) > db.debounceDelay) {
        if (reading != db.stableState) {
            db.stableState = reading;
            return (db.stableState == LOW) ? 1 : -1;
        }
    }
    return 0;
}

void buttonInit(button_t* b) {
    pinMode(b->pin, b->io);
    b->buttonState = digitalRead(b->pin);
    b->lastButtonState = b->buttonState;
}

// not used later in the code
void buttonUpdate(button_t* b) {
    b->lastButtonState = b->buttonState;
    b->buttonState = digitalRead(b->pin);
}

void Buttons::setup() {
    analogReadResolution(12);
    pinMode(buttonPins::powerButton.pin, buttonPins::powerButton.io);
    pinMode(buttonPins::toolSelect.pin, buttonPins::toolSelect.io);
    pinMode(buttonPins::triggerInput.pin, buttonPins::triggerInput.io);
    dbPower.lastReading = dbPower.stableState = digitalRead(buttonPins::powerButton.pin);
    dbCycle.lastReading = dbCycle.stableState = digitalRead(buttonPins::toolSelect.pin);
}

bool powerButtonWasPressed() {
    return pollEdge(&buttonPins::powerButton, dbPower) == 1;
}

bool toolSelectWasPressed() {
    return pollEdge(&buttonPins::toolSelect, dbCycle) == 1;
}

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