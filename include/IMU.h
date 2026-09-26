// BNO085 IMU reads (orientation, gyro, accel), plus the IMU-to-console 
// mapping, re-zero/re-center handling stuff, and drift tracking system
// for the handle's ball-and-socket mount.
//
// For: Handle ESP32


#pragma once

// Angles are in degrees.
struct ImuOrientation {
    float roll;
    float pitch;
    float yaw;
};

bool initIMU();
void updateIMU();
bool imuHasOrientation();
ImuOrientation getIMUOrientation();
