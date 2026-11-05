// Bench test: sends real BNO085 orientation to the Pi over the UDP link at 10 Hz. Run
// base/handle_link_test on the Pi to see roll/pitch/yaw in its terminal.
// Build with the handle-imu-link-test env. IMU wiring is listed in imu_test.cpp.
//
// For: Handle ESP32-S3

#include <Arduino.h>
#include "base_link.h"
#include "imu.h"

static unsigned long lastSendMs = 0, lastPrintMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // WiFi first: the BNO streams from initIMU() on, and nothing reads it during a slow WiFi join.
    Serial.println("Connecting...");
    if (!base_link_init())
        Serial.println("WiFi FAILED (check SSID/password, must be 2.4 GHz)");

    Serial.println("Starting BNO085...");
    if (!initIMU())
    {
        Serial.println("BNO085 initialization failed: no reply on UART.");
        while (true)
        {
            delay(1000);
        }
    }
    Serial.println("BNO085 connected");
}

void loop()
{
    base_link_update();
    updateIMU();
    unsigned long now = millis();

    // Send at 10 Hz. Gyro and accel stay zero until those BNO reports are enabled.
    if (imuHasOrientation() && now - lastSendMs >= 100)
    {
        lastSendMs = now;
        ImuOrientation angles = getIMUOrientation();

        handleStatePayload s;
        memset(&s, 0, sizeof(s));
        s.timestampMs = now;
        for (int i = 0; i < 4; ++i)
            s.quat[i] = angles.quat[i];
        base_link_send_state(s);
    }

    // Local copy of the angles, to compare against the Pi's terminal
    if (now - lastPrintMs >= 1000)
    {
        lastPrintMs = now;
        ImuOrientation angles = getIMUOrientation();
        if (imuHasOrientation())
            Serial.printf("link %s  roll=%.1f pitch=%.1f yaw=%.1f\n", base_link_is_up() ? "UP" : "DOWN",
                          angles.roll, angles.pitch, angles.yaw);
        else
            Serial.printf("link %s  waiting for IMU data\n", base_link_is_up() ? "UP" : "DOWN");
    }
}
