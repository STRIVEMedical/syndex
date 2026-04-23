// joint.h - Simple joint mapping for 7DOF arm
#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "odrive.h"

#define INACTIVE_CHANNEL 255
#define NUM_JOINTS 7

// Stores hardware mapping for one joint

struct Joint {
    // Configuration
    ODriveCAN*      odrive;         // null if no ODrive on this joint
    ODriveUserData* user_data;      // null if no ODrive on this joint
    uint8_t         sensor_channel;  // I2C mux channel — INACTIVE_CHANNEL if not used (i.e joint uses odrive)
    float           max_torque;     // safety limit (Nm)
    float           home_pos;     // home position (turns)
    bool            use_onboard_encoder; // true = read pos from ODrive, not AS5600
    const char*     label;          // debug name

    // Runtime state
    float           angle;          // latest joint angle (deg)
    uint16_t        rawValue;       // latest raw encoder reading (0-4095, 0xFFFF on read failure)
    float           velocity;        // latest velocity estimate from odrive 
    bool            is_homed;       // joint homing status
    float           target_torque;  // commanded torque target (Nm)
    float           max_position;   // 
    float           min_position;    // 
};

// Initialize all joints with default mapping
void initJoints();

// Get joint by ID
Joint* getJoint(uint8_t id);

// Read all joint angles and raw value from AS5600 encoders
void readJointAngles();

// Print joint status
void printJointStatus();

// Returns true if all ODrive-backed joints have had their zero reference established via confirmHome().
bool isHomed();

// Latches the current encoder position as 0 on all active ODrives and marks joints homed.
// Call this after the operator has physically placed the arm at the home pose.
void confirmHome();

#endif // JOINT_H