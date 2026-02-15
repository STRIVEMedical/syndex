// joint.h - Simple joint mapping for 7DOF arm
#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "odrive.h"

#define NUM_JOINTS 2 

// Stores hardware mapping for one joint
struct Joint {
    // Hardware mapping
    ODriveCAN* odrive;        // Which ODrive controls this joint (0-6)
    uint8_t sensor_channel;   // Which I2C mux channel reads this joint (0-7)
    
    // Current state
    float angle;              // Current angle in degrees
    float rawValue;           //raw value from encoders
    bool is_homed;            // Has joint been homed?
    float target_torque;      // Torque to apply (Nm)
    
    // Configuration
    float home_angle;         // Home position in degrees
    float max_torque;         // Safety limit in Nm
};

// Initialize all joints with default mapping
void initJoints();

// Get joint by ID
Joint* getJoint(uint8_t id);

// Read all joint angles and raw value from AS5600 encoders
void readJointAnglesAndRaw();

// Print joint status
void printJointStatus();

#endif // JOINT_H