#include "buttons.h"

namespace buttonPins {
    button_t powerButton   = {0,  INPUT_PULLUP, HIGH, HIGH};
    button_t autoHoming    = {14, INPUT_PULLUP, HIGH, HIGH};
    button_t triggerButton = {15, INPUT_PULLUP, HIGH, HIGH};
    button_t toolSelect    = {16, INPUT_PULLUP, HIGH, HIGH};
}

class Buttons {
    void setup() {
        buttonInit(&buttonPins::powerButton);
        buttonInit(&buttonPins::autoHoming);
        buttonInit(&buttonPins::triggerButton);
        buttonInit(&buttonPins::toolSelect);
    }
}

void pinSetup(button_t* b) {
    pinMode(b->pin, b->io);
}

void buttonDetect(button_t* b)                     
{
  if (digitalRead(b->pin) == LOW) {
    Serial.println("Pin %d button pressed", b.pin)
  }
  delay(100);
}

void buttonUpdate(button_t* b) {
    b->lastState = b->state;
    b->state = digitalRead(b->pin);
}

void buttonInit(button_t* b) {
    pinMode(b->pin, b->mode);
    b->state = digitalRead(b->pin);
    b->lastState = b->state;
}