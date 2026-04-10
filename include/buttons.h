#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>

typedef struct {
    uint8_t pin; // GPIO Pin #
    uint8_t io; // INPUT or INPUT_PULLUP
    uint8_t buttonState; // Current button state
    uint8_t lastButtonState; // Last state from button, active LOW
} button_t;

namespace buttonPins {
    extern button_t powerButton;
    extern button_t toolSelect;
    extern button_t triggerInput;
}

typedef struct {
  bool power_pressed  = false;
  bool power_released = false;

  bool tool_pressed   = false;
  bool tool_released  = false;

  bool trigger_pressed  = false;
  bool trigger_released = false;

} ButtonEvents_t;

void buttonInit(button_t* b);

void buttonDetect(button_t* b);

void buttonUpdate(button_t* b);

namespace Buttons {
    void setup();
};

void ToolCycleButton(button_t* b);
void testPowerButton(button_t* b);
float getTriggerDepth(button_t* b);
void triggerPulled(button_t* b);

#endif