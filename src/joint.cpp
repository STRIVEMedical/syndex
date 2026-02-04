#include "joint.h"
#include "i2c.h"
#include "odrive.h"

// Array of all joints
static Joint joints[NUM_JOINTS];

// Map an ODrive pointer to a readable label.
static const char* odriveLabel(const ODriveCAN* odrv) {
    if (odrv == &odrv0) return "odrv0";
    if (odrv == &odrv1) return "odrv1";
    // if (odrv == &odrv2) return "odrv2";
    // if (odrv == &odrv3) return "odrv3";
    // if (odrv == &odrv4) return "odrv4";
    // if (odrv == &odrv5) return "odrv5";
    // if (odrv == &odrv6) return "odrv6";
    return "unknown";
}

// Initialize joints with default mapping
//CHANGE VALUES TO MATCH HARDWARE
void initJoints() {
    // Default mapping - CHANGE THESE TO MATCH YOUR ARM!
    // Format: {odrive_id, sensor_channel, angle, raw_value, is_homed, target_torque, home_angle, max_torque}
    

    joints[0] = {&odrv0, 0, 0.0, 0.0, false, 0.0, 0.0, 5.0};   // pulley1
    joints[1] = {&odrv1, 1, 0.0, 0.0, false, 0.0, 0.0, 5.0};   // pulley2
    // joints[2] = {&odrv2, 2, 0.0, 0.0, false, 0.0, 90.0, 3.0};  // arm_3
    // joints[3] = {&odrv3, 3, 0.0, 0.0, false, 0.0, 0.0, 2.0};   // part_18
    // joints[4] = {&odrv4, 4, 0.0, 0.0, false, 0.0, 0.0, 1.0};   // manipulatortestassem
    // joints[5] = {&odrv5, 5, 0.0, 0.0, false, 0.0, 0.0, 0.5};   // (wrist joint)
    // joints[6] = {&odrv6, 6, 0.0, 0.0, false, 0.0, 0.0, 0.5};   // (tool joint)
    
    Serial.println("Joints initialized with configured mapping");
}

// Get joint pointer by ID
Joint* getJoint(uint8_t id) {
    if (id >= NUM_JOINTS){
        return nullptr;
    }

    return &joints[id];
}

// Read all joint angles and raw value from AS5600 encoders
void readJointAnglesAndRaw() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        // Select the correct I2C mux channel for this joint
        tcaSelect(joints[i].sensor_channel);
        delayMicroseconds(200);
        
        // Read raw encoder value
        joints[i].rawValue  = readRawAS5600();
        // Convert to angle and store in joint
        joints[i].angle = computeAngle(i, joints[i].rawValue); 
    }
}

// Print joint status to serial
void printJointStatus() {
    Serial.println("=== Joint Status ===");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print("J");
        Serial.print(i);
        Serial.print(": ODrive");
        Serial.print(odriveLabel(joints[i].odrive));
        Serial.print(", SensorCH");
        Serial.print(joints[i].sensor_channel);
        Serial.print(", Angle=");
        Serial.print(joints[i].angle, 1);
        Serial.print("°, Home=");
        Serial.print(joints[i].home_angle, 0);
        Serial.print("°, Homing=");
        Serial.print(joints[i].is_homed ? "YES" : "NO");
        Serial.print(", Torque=");
        Serial.print(joints[i].target_torque, 2);
        Serial.println("Nm");
    }
    Serial.println("====================");
}