#include <Arduino.h>
#include "IMU.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!initIMU()) {
        Serial.println("BNO085 initialization failed");
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

        Serial.printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
                      angles.roll, angles.pitch, angles.yaw);
    }
}