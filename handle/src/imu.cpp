// File includes rotation for handles

#include "imu.h"

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

// Adafruit breakout in UART (SHTP) mode: P1 tied to 3V3, P0 left low. With both low it boots in I2C.
#define BNO_RX_PIN 16  // ESP32 RX <- BNO085 TX (breakout SDA)
#define BNO_TX_PIN 17  // ESP32 TX -> BNO085 RX (breakout SCL)

namespace {

HardwareSerial imuSerial(1);    // setup serial comms
Adafruit_BNO08x sensor(-1);  // No reset GPIO connected

ImuOrientation orientation{}; // create orientation  struct
bool hasOrientation = false;
bool connected = false;

// The library's UART reads block until bytes arrive, so begin_UART() hangs instead of failing
// if the BNO is unpowered, miswired or not strapped for UART. Send the same soft reset the
// library sends and give the BNO 1 s to answer first.
bool bnoAnswersOnUart() {
    while (imuSerial.available()) {
        imuSerial.read();
    }

    const uint8_t softReset[] = {0x7E, 0x01, 0x05, 0x00, 0x01, 0x00, 0x01, 0x7E};
    for (uint8_t b : softReset) {
        imuSerial.write(b);
        delay(1);  // BNO needs gaps between host bytes
    }

    const unsigned long start = millis();
    while (millis() - start < 1000) {
        if (imuSerial.available() && imuSerial.read() == 0x7E) {
            delay(100);  // let it finish its startup packets
            return true;
        }
        delay(1);
    }
    return false;
}

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

    // make sure the BNO is talking before the library's blocking reads take over
    if (!bnoAnswersOnUart()) {
        connected = false;
        hasOrientation = false;
        return false;
    }

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
    // (UART reads block until a full packet arrives, so only read what's already buffered)
    while (imuSerial.available() > 0 && sensor.getSensorEvent(&value)) {
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