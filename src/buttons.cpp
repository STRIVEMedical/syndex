#include "buttons.h"

#define TRIGGER_MIN_ADC 0
#define TRIGGER_MAX_ADC 4046

namespace buttonPins {
    button_t powerButton = {39, INPUT, HIGH, HIGH};
    // button_t toolSelect  = {39, INPUT, HIGH, HIGH};
    // INPUT: analog read with threshold. Resting voltage ~1.7V, pressed = 3.3V.
    // Threshold set at ~2.5V (3100/4096 counts) in isTriggerPressed().
    button_t triggerInput = {40, INPUT, LOW, LOW};
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
            // Invert logic: treat HIGH as pressed
            return (db.stableState == HIGH) ? 1 : -1;
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
// C3AW-1A-8F is a snap-action switch (boolean only — no analog depth).
// Wiring: one wire to pin 40, other wire to GND.
// Pin configured INPUT_PULLUP → Unpressed = HIGH, Pressed = LOW (active-low).

// Legacy analog stub kept so callers that reference getTriggerDepth() still compile.
// Always returns 0.0 — use isTriggerPressed() for the real reading.
float getTriggerDepth(button_t* b) {
    (void)b;
    return 0.0f;
}

// Returns true while the trigger is held (debounced).
// Uses analogRead because the resting voltage is ~1.7V (above the digital HIGH
// threshold), so digitalRead would report pressed at rest. Threshold set at
// 3100/4096 (~2.5V) — safely above the 1.7V float and below the 3.3V press level.
// 12-bit ADC (0-4095): unpressed ~2120, pressed ~4040. Threshold at midpoint.
#define TRIGGER_PRESS_THRESHOLD 3000

bool isTriggerPressed(button_t* b) {
    static int   lastReading      = LOW;
    static int   stableState      = LOW;
    static unsigned long lastDebounceTime = 0;
    const  unsigned long DEBOUNCE_MS      = 20;

    int reading = (analogRead(b->pin) >= TRIGGER_PRESS_THRESHOLD) ? HIGH : LOW;
    if (reading != lastReading) {
        lastDebounceTime = millis();
    }
    lastReading = reading;

    if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
        stableState = reading;
    }

    return (stableState == LOW);   // LOW = switch closed = pressed
}

void triggerPulled(button_t* b) {
    static bool wasPressed = false;

    bool pressed = isTriggerPressed(b);

    if (!wasPressed && getTriggerDepth(b) > 0.05f) {
        if (!wasPressed) {
            Serial.println("Trigger Pressed");
            wasPressed = true;
        }
    } else {
        if (wasPressed && getTriggerDepth(b) < 0.02F) {
            Serial.println("Trigger Released");
            wasPressed = false;
        }
    }
}