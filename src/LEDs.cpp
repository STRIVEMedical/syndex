#include "LEDs.h"
#include "states.h"

#define FALSE 0
#define TRUE 1

namespace Led {
    /* Receiving satisfactory power for Teensy operation */
    led_t powerLed = {7, POWER};

    /* Receiving data from encoders/ODrives, transitively ready to send data to PC */
    led_t dataLed = {8, DATA};

    /* Toggle errorLed on an error mode */
    led_t errorLed = {28, ERROR};

    bool transferData = false;

    void setup() {
        initLED(&Led::powerLed);
        initLED(&Led::dataLed);
        initLED(&Led::errorLed);
    }
}

// ─── LED Drivers ─────────────────────────────────────────────────────────────

void initLED(led_t* l) {
    pinMode(l->pin, OUTPUT);
}

void ONToggleLED(led_t* l) {
    digitalWrite(l->pin, HIGH);
}

void OFFToggleLED(led_t* l) {
    digitalWrite(l->pin, LOW);
}

// ─── Silent Probe (no side effects) ──────────────────────────────────────────

/*
 * Probes all 7 I2C encoder channels WITHOUT calling setError().
 * Used exclusively for LED status checks so a transient read hiccup
 * doesn't corrupt currError during normal operation.
 * Returns true if all encoders respond, false if any channel fails.
 */
static bool probeI2CDevices() {
    for (uint8_t ch = 0; ch < 7; ch++) {
        tcaSelect(ch);
        uint16_t raw = readRawAS5600();
        if (raw == 0xFFFF) {
            return false;   // encoder not responding — silent, no setError()
        }
    }
    return true;
}

// ─── Data LED ─────────────────────────────────────────────────────────────────

/*
 * Evaluates whether any of the three data transfer layers are active
 * and writes the result into Led::transferData for use by Update().
 *
 * Layers checked (OR logic — any one active = data flowing):
 *   1. USB Serial  — host PC  <-> Teensy
 *   2. CAN         — Teensy   <-> ODrives
 *   3. I2C         — Teensy   <-> AS5600 encoders via TCA9548A mux
 *
 * Uses probeI2CDevices() instead of verifyConnectedI2CDevices() to avoid
 * setting currError as a side effect of an LED poll.
 * Call once per loop cycle before Update().
 */
void updateDataLED() {
    bool usbActive    = Serial;                          // USB host connected
    bool odriveActive = odrv0_user_data.received_heartbeat
                     && odrv1_user_data.received_heartbeat; // CAN heartbeats alive
    bool i2cActive    = probeI2CDevices();               // all 7 encoders responding

    Led::transferData = usbActive || odriveActive || i2cActive;
}

// ─── LED State Update ─────────────────────────────────────────────────────────

/*
 * Drives all three LED outputs based on current system state.
 * Call once per loop cycle AFTER updateDataLED().
 *
 * Power LED  — always ON while Update() is running (system alive)
 * Data LED   — blinks at 200 ms intervals while Led::transferData is true,
 *              OFF when no data transfer is active
 * Error LED  — managed externally by turnOnErrorLED() / errorRecovery()
 *              (not touched here to avoid fighting ERROR_STATE logic)
 */
void Update() {
    static uint32_t lastToggleTime = 0;  // tracks last blink edge in ms
    static bool     dataLEDState   = false; // current blink state of data LED

    // Power LED — solid ON as long as the system is running
    ONToggleLED(&Led::powerLed);

    // Data LED — blink while data is transferring, off when idle
    if (Led::transferData) {
        uint32_t now = millis();
        if (now - lastToggleTime >= 200) {  // toggle every 200 ms
            dataLEDState   = !dataLEDState;
            lastToggleTime = now;
            if (dataLEDState) {
                ONToggleLED(&Led::dataLed);
            } else {
                OFFToggleLED(&Led::dataLed);
            }
        }
    } else {
        // No data flowing — hold LED off and reset blink state
        OFFToggleLED(&Led::dataLed);
        dataLEDState = false;
    }
}

// ─── Connection Verification (fault-reporting, used by state machine) ─────────

bool verifyUSB() {
    if (!Serial) {
        setError(CONNECTION_ERROR);
        return false;
    }
    return true;
}

bool verifyODriveComms() {
    if (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        return false;
    }
    return true;
}

bool verifyConnectedI2CDevices() {
    for (uint8_t ch = 0; ch < 7; ch++) {
        tcaSelect(ch);
        uint16_t raw = readRawAS5600();
        if (raw == 0xFFFF) {
            setError(I2C_ERROR);
            return false;
        }
    }
    return true;
}

/*
 * Convenience wrapper that checks all three connection layers are healthy.
 * Checks in order: USB → ODrive CAN → I2C encoders
 * Sets specific error code for whichever layer fails first.
 * Returns true only if all three layers pass, false if any fail.
 */
bool allConnectionsReady() {
    if (!verifyUSB())                  { setError(CONNECTION_ERROR); return false; }
    if (!verifyODriveComms())          { setError(ODRIVE_ERROR);     return false; }
    if (!verifyConnectedI2CDevices())  { setError(I2C_ERROR);        return false; }
    return true;
}




void testLEDs() { //testLEDS

    Serial.println("Testing LEDs...");

    ONToggleLED(&Led::powerLed);
    Serial.println("Power LED ON");
    delay(1000);
    OFFToggleLED(&Led::powerLed);
    Serial.println("Power LED OFF");
    delay(500);

    ONToggleLED(&Led::dataLed);
    Serial.println("Data LED ON");
    delay(1000);
    OFFToggleLED(&Led::dataLed);
    Serial.println("Data LED OFF");
    delay(500);

    ONToggleLED(&Led::errorLed);
    Serial.println("Error LED ON");
    delay(1000);
    OFFToggleLED(&Led::errorLed);
    Serial.println("Error LED OFF");
    delay(500);

    Serial.println("LED test complete.");
} 
