// Bench test for the BNO085 over UART: prints roll/pitch/yaw and heading accuracy at 10 Hz.
// Wiring (Adafruit breakout): VIN->3V3, GND->G, SDA->GPIO16, SCL->GPIO17, P1->3V3, P0 open.
// Build with the handle-imu-test env; monitor on the DevKit's USB-to-UART port at 115200.
//
// For: Handle ESP32-S3

#include <Arduino.h>
#include "imu.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting BNO085 over UART (RX=GPIO16, TX=GPIO17)...");
    if (!initIMU()) {
        Serial.println("BNO085 initialization failed: no reply on UART.");
        Serial.println("Check P1 -> 3V3 (P0 open), SDA -> GPIO16, SCL -> GPIO17, VIN -> 3V3, then power-cycle.");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("BNO085 connected");
}

void loop() {
    updateIMU();

    static unsigned long lastPrint = 0;
    if (imuHasOrientation() && millis() - lastPrint >= 100) {
        lastPrint = millis();
        ImuOrientation angles = getIMUOrientation();

        Serial.printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f  Acc: %.1f deg\n",
                      angles.roll, angles.pitch, angles.yaw, angles.accuracyRad * RAD_TO_DEG);
    }
}
