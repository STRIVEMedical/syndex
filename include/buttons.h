#include <Arduino.h>

typdef struct {
    uint8_t pin; // GPIO Pin #
    uint8_t io; // INPUT or INPUT_PULLUP
    uint8_t buttonState; // Current button state
    uint8_t lastButtonState = HIGH; // Last state from button, active LOW
} button_t;

namespace buttonPins {
    extern button_t powerButton;
    extern button_t autoHoming;
    extern button_t triggerButton;
    extern button_t toolSelect;
}

typedef struct {
  bool power_pressed  = false;
  bool power_released = false;

  bool item_pressed   = false;
  bool item_released  = false;

  bool trigger_pressed  = false;
  bool trigger_released = false;

  bool home_pressed   = false;
  bool home_released  = false;
} ButtonEvents_t;

void buttonInit();
void pinSetup();
void buttonDetect();
void buttonUpdate();

class Buttons {
    void setup();
}