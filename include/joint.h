#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "odrive.h"

#define NUM_JOINTS 7

struct Joint {
    ODriveCAN*      odrive;         // null if no ODrive on this joint
    ODriveUserData* user_data;      // null if no ODrive on this joint
    uint8_t         sensor_channel;  // I2C mux channel — ignored if use_onboard_encoder
    float           max_torque;     // safety limit (Nm)
    float           home_angle;     // home position (deg)
    bool            has_odrive;     // false = encoder only joint
    bool            use_onboard_encoder; // true = read pos from ODrive, not AS5600
    const char*     label;          // debug name
};


extern Joint joints[NUM_JOINTS];

void        initJoints();
void        readJointAngles();
float       getJointAngle(uint8_t id);      // returns degrees
bool        isJointHomed(uint8_t id);
void        printJointStatus();

#endif