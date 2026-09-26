// This will include the IMU logic needed for sensing the positions (xyz)

#include "IMU.h"

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

#define BNO_RX_PIN 16  // ESP32 RX <- BNO085 TX
#define BNO_TX_PIN 17  // ESP32 TX -> BNO085 RX

namespace {
HardwareSerial imuSerial(1);
Adafruit_BNO08x sensor(-1);  // No reset GPIO connected

ImuOrientation orientation{};
bool hasOrientation = false;
bool connected = false;

bool enableOrientationReport() {
    return sensor.enableReport(SH2_ROTATION_VECTOR, 10000);  // 100 Hz
}

void convertQuaternion(float w, float x, float y, float z) {
    const float roll = atan2f(
        2.0f * (w * x + y * z),
        1.0f - 2.0f * (x * x + y * y)
    );

    const float sinPitch = 2.0f * (w * y - z * x);
    const float pitch = asinf(constrain(sinPitch, -1.0f, 1.0f));

    const float yaw = atan2f(
        2.0f * (w * z + x * y),
        1.0f - 2.0f * (y * y + z * z)
    );

    constexpr float degrees = 180.0f / PI;
    orientation = {
        roll * degrees,
        pitch * degrees,
        yaw * degrees
    };
    hasOrientation = true;
}
}  // namespace

bool initIMU() {
    imuSerial.setRxBufferSize(1024);
    imuSerial.begin(3000000, SERIAL_8N1, BNO_RX_PIN, BNO_TX_PIN);

    connected = sensor.begin_UART(&imuSerial);
    if (!connected) {
        return false;
    }

    connected = enableOrientationReport();
    return connected;
}

void updateIMU() {
    if (!connected) {
        return;
    }

    // The sensor needs its reports enabled again after an internal reset.
    if (sensor.wasReset() && !enableOrientationReport()) {
        connected = false;
        return;
    }

    sh2_SensorValue_t value;
    while (sensor.getSensorEvent(&value)) {
        if (value.sensorId == SH2_ROTATION_VECTOR) {
            const auto& q = value.un.rotationVector;
            convertQuaternion(q.real, q.i, q.j, q.k);
        }
    }
}

bool imuHasOrientation() {
    return hasOrientation;
}

ImuOrientation getIMUOrientation() {
    return orientation;
}