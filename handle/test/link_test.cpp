#include <Arduino.h>
#include "base_link.h"

#ifndef MOTOR_PIN
#define MOTOR_PIN 4 // placeholder: set to your real motor driver pin
#endif

static unsigned long lastSendMs = 0, lastPrintMs = 0, motorOffAt = 0;
static bool motorOn = false;

static void motorSet(bool on)
{
    motorOn = on;
    digitalWrite(MOTOR_PIN, on ? HIGH : LOW);
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(MOTOR_PIN, OUTPUT);
    motorSet(false);

    Serial.println("Connecting...");
    if (!base_link_init())
        Serial.println("WiFi FAILED (check SSID/password, must be 2.4 GHz)");
}

void loop()
{
    base_link_update();
    unsigned long now = millis();

    // Send fake state at 10 Hz
    if (now - lastSendMs >= 100)
    {
        lastSendMs = now;
        handleStatePayload s;
        memset(&s, 0, sizeof(s));
        s.timestampMs = now;
        s.quat[0] = 1.0f;
        base_link_send_state(s);
    }

    // Vibration command from the base
    vibrateCmdPayload v;
    if (base_link_get_vibrate(v))
    {
        Serial.printf("VIBRATE cmd seq=%u intensity=%u duration=%ums\n",
                      v.meta.seq, v.intensity, v.durationMs);
        if (v.intensity > 0)
        {
            motorSet(true);
            motorOffAt = now + v.durationMs;
        }
        else
            motorSet(false);
    }

    // Stop after duration expires, and ALWAYS stop on link loss
    if (motorOn && (long)(now - motorOffAt) >= 0)
        motorSet(false);
    if (!base_link_is_up() && motorOn)
        motorSet(false);

    if (now - lastPrintMs >= 1000)
    {
        lastPrintMs = now;
        Serial.printf("link %s\n", base_link_is_up() ? "UP" : "DOWN");
    }
}