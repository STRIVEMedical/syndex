// File includes rotation for handles

#include "IMU.h"

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

#define BNO_RX_PIN 16  // ESP32 RX <- BNO085 TX
#define BNO_TX_PIN 17  // ESP32 TX -> BNO085 RX

namespace {

HardwareSerial imuSerial(1);    // setup serial comms
Adafruit_BNO08x sensor(-1);  // No reset GPIO connected

ImuOrientation orientation{}; // create orientation  struct
bool hasOrientation = false;
bool connected = false;

// is orientation enabled
bool enableOrientationReport() {
    return sensor.enableReport(SH2_ROTATION_VECTOR, 10000);  // 100 Hz
}

// take in 5 vals to return roll pitch yaw
void convertQuaternion(float w, float x, float y, float z, float accuracyRad) {
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
    orientation.roll = roll * degrees;
    orientation.pitch = pitch * degrees;
    orientation.yaw = yaw * degrees;
    orientation.accuracyRad = accuracyRad;
    hasOrientation = true;
}
}
bool initIMU() {
    // buffer on ESP set to 1024
    imuSerial.setRxBufferSize(1024);
    // configure esp comms
    imuSerial.begin(3000000, SERIAL_8N1, BNO_RX_PIN, BNO_TX_PIN);

    // initalize sensor on ESP uart channel
    connected = sensor.begin_UART(&imuSerial);
    // if its not connected you cant initalize
    if (!connected) {
        hasOrientation = false;
        return false;
    }

    //get data up and moving
    connected = enableOrientationReport();
    if (!connected) {
        hasOrientation = false;
    }
    return connected;
}
// will update the roll pitch yaw of handle
void updateIMU() {
    // no connect = no work
    if (!connected) {
        return;
    }

    // if the sensor has been reset then it cant update values
    if (sensor.wasReset()) {
        hasOrientation = false;
            // get out of update and update to disconnected
        if (!enableOrientationReport()) {
            connected = false;
            return;
        }
    }
    // create container for data to be put in before compilation
    sh2_SensorValue_t value;

    // while there is data to be got, get data
    while (sensor.getSensorEvent(&value)) {
        // make sure its rotation data
        if (value.sensorId == SH2_ROTATION_VECTOR) {
            // put containerdata into something we can use
            const auto& q = value.un.rotationVector;
            // math
            convertQuaternion(q.real, q.i, q.j, q.k, q.accuracy);
        }
    }
}

// check if orientation
bool imuHasOrientation() {
    return hasOrientation;
}

// return orientiation
ImuOrientation getIMUOrientation() {
    return orientation;
}