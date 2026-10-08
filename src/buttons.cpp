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

static DebouncedButton dbPower = {HIGH, HIGH, 0};

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

bool powerButtonWasPressed() {
    return pollEdge(&buttonPins::powerButton, dbPower) == 1;
}

/* ========== TRIGGER CODE ========== */
// C3AW-1A-8F is a snap-action switch (boolean only; no analog depth).
// Read through ADC because this board sees unpressed around 1.7V and pressed
// near 3.3V. Higher ADC values are treated as pressed.

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

    return (stableState == HIGH);
}
