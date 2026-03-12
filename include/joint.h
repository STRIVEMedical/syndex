#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "odrive.h"

#define NUM_JOINTS 3

// Read-only hardware mapping for one joint
struct Joint {
    ODriveCAN*      odrive;         // which ODrive drives this joint
    ODriveUserData* user_data;      // CAN feedback for this joint
    uint8_t         sensor_channel; // I2C mux channel for AS5600
    float           max_torque;     // safety limit (Nm)
    float           home_angle;     // home position (deg)
    const char*     label;          // debug name
};

extern Joint joints[NUM_JOINTS];

void        initJoints();
void        readJointAngles();
float       getJointAngle(uint8_t id);      // returns degrees
bool        isJointHomed(uint8_t id);
void        printJointStatus();

#endif