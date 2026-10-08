// Debounced digital-input reads for the handle's physical controls.
//
// Fpr: Handle ESP32

#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>

typedef struct {
    uint8_t pin; // GPIO Pin #
    uint8_t io; // INPUT or INPUT_PULLUP
    uint8_t buttonState; // Current button state
    uint8_t lastButtonState; // Last state from button, active LOW
} button_t;

struct DebouncedButton {
    int lastReading;
    int stableState;
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 50;
};

namespace buttonPins {
    extern button_t powerButton;
    extern button_t toolSelect;
    extern button_t triggerInput;
}

bool  isTriggerPressed(button_t* b);  // true while trigger is held (debounced, active-low)
bool powerButtonWasPressed();

// V1's buttons.cpp defines Buttons::setup() but the Buttons scope was never declared
// (compile error). This declaration is the only change made when porting the file.
namespace Buttons {
    void setup();
}

#endif
