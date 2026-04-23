#include "buttons.h"

#define TRIGGER_MIN_ADC 0
#define TRIGGER_MAX_ADC 4046

namespace buttonPins {
    button_t powerButton = {37, INPUT, HIGH, HIGH};
    button_t toolSelect  = {39, INPUT, HIGH, HIGH};
    button_t triggerInput = {40, INPUT, HIGH, HIGH};
}

struct DebouncedButton {
    int lastReading;
    int stableState;
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 50;
};

static DebouncedButton dbPower = {HIGH, HIGH, 0};
static DebouncedButton dbCycle = {HIGH, HIGH, 0};

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