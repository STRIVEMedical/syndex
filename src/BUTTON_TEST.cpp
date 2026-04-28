#include <Arduino.h>
#include "buttons.h"

// ─── LED Pins (raw — no LEDs.h dependency) ───────────────────────────────────
// Matched to your pinout: Green=15, Yellow=14, Red=13
//#define LED_GREEN  15
//#define LED_YELLOW 14
//#define LED_RED    13

// ─── Trigger Input (pin 40, analog) ──────────────────────────────────────────

// ─── Button Poll ──────────────────────────────────────────────────────────────
static void pollButton(button_t* b, DebouncedButton& db, const char* label) {
    int reading = digitalRead(b->pin);
    unsigned long now = millis();

    if (reading != db.lastReading) {
        db.lastDebounceTime = now;
    }

    if ((now - db.lastDebounceTime) > db.debounceDelay) {
        if (reading != db.stableState) {
            db.stableState = reading;

            if (db.stableState == LOW) {
                Serial.print(label);
                Serial.println(" was released");
            } else {
                Serial.print(label);
                Serial.println(" was pressed");
            }
        }
    }

    db.lastReading = reading;
}
// static void pollButton(button_t* b, DebouncedButton& db, const char* label) {
//     int reading = digitalRead(b->pin);

//     if (reading != db.lastReading) {
//         db.lastDebounceTime = millis();
//     }

//     if ((millis() - db.lastDebounceTime) > db.debounceDelay) {
//         if (reading != db.stableState) {
//             db.stableState = reading;

//             if (db.stableState == LOW) {
//                 Serial.print(label);
//                 Serial.println(" was pressed");
//             } else {
//                 Serial.print(label);
//                 Serial.println(" was released");
//             }
//         }
//     }

//     db.lastReading = reading;
// }

// ─── Trigger Poll ─────────────────────────────────────────────────────────────

// static void pollTrigger(button_t* b) {
//     static bool wasPressed = false;
//     float depth = getTriggerDepth(b);

//     if (depth > 0.01f) {
//         if (!wasPressed) {
//             Serial.println("Trigger was pressed");
//             wasPressed = true;
//         }
//     } else {
//         if (wasPressed) {
//             Serial.println("Trigger was released");
//             wasPressed = false;
//         }
//     }
// }

// // ─── LED Test (runs once at startup) ─────────────────────────────────────────

// static void runLEDTest() {
//     Serial.println("=== LED Test ===");

//     digitalWrite(LED_GREEN, HIGH);
//     Serial.println("Green LED ON");
//     delay(1000);
//     digitalWrite(LED_GREEN, LOW);
//     Serial.println("Green LED OFF");
//     delay(500);

//     digitalWrite(LED_YELLOW, HIGH);
//     Serial.println("Yellow LED ON");
//     delay(1000);
//     digitalWrite(LED_YELLOW, LOW);
//     Serial.println("Yellow LED OFF");
//     delay(500);

//     digitalWrite(LED_RED, HIGH);
//     Serial.println("Red LED ON");
//     delay(1000);
//     digitalWrite(LED_RED, LOW);
//     Serial.println("Red LED OFF");
//     delay(500);

//     Serial.println("=== LED Test Complete ===");
// }

// ─── Arduino Entry Points ─────────────────────────────────────────────────────

void buttonTestSetup() {
    Serial.begin(115200);
    //while (!Serial) { ; }  

    delay(500);

    // // LED pins
    // pinMode(LED_GREEN, OUTPUT);
    // pinMode(LED_YELLOW, OUTPUT);
    // pinMode(LED_RED, OUTPUT);

    // // Buttons + trigger
    Buttons::setup(); 
    delay(100); 
    // analogReadResolution(12);
    // pinMode(triggerInput.pin, INPUT);

    unsigned long start = millis();
    while (millis() - start < 500) {
        dbPower.lastReading = dbPower.stableState = digitalRead(buttonPins::powerButton.pin);
        dbCycle.lastReading = dbCycle.stableState = digitalRead(buttonPins::toolSelect.pin);
    }

    Serial.println("=== Button Test Ready ===");
    Serial.println("Press power button, cycle button, or pull trigger.");
    Serial.println();
    Serial.print("Trigger ADC at rest: ");
    Serial.println(analogRead(buttonPins::triggerInput.pin));
    //testPowerButton(&buttonPins::powerButton);
    //ToolCycleButton(&buttonPins::toolSelect);


    //runLEDTest();
}

void buttonTestLoop() {
    pollButton(&buttonPins::powerButton, dbPower, "Power button");
    pollButton(&buttonPins::toolSelect,  dbCycle, "Cycle button");
    triggerPulled(&buttonPins::triggerInput);
    //pollTrigger(&triggerInput);
}