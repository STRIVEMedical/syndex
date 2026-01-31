#include "buttons.h"

namespace buttonPins {
    button_t powerButton   = {0,  INPUT_PULLUP, HIGH, HIGH};
    button_t autoHoming    = {14, INPUT_PULLUP, HIGH, HIGH};
    button_t triggerButton = {15, INPUT_PULLUP, HIGH, HIGH};
    button_t toolSelect    = {16, INPUT_PULLUP, HIGH, HIGH};
}

void buttonInit(button_t* b) {
    pinMode(b->pin, b->io);
    b->buttonState = digitalRead(b->pin);
    b->lastButtonState = b->buttonState;
}

void Buttons::setup() {
    buttonInit(&buttonPins::powerButton);
    buttonInit(&buttonPins::autoHoming);
    buttonInit(&buttonPins::triggerButton);
    buttonInit(&buttonPins::toolSelect);
};

void buttonDetect(button_t* b)                     
{
  if (digitalRead(b->pin) == LOW) {
    Serial.print("\nPin button pressed: ");
    Serial.print(b->pin);
  }
  delay(100);
}

void buttonUpdate(button_t* b) {
    b->lastButtonState = b->buttonState;
    b->buttonState = digitalRead(b->pin);
}