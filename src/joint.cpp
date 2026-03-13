#include "joint.h"
#include "i2c.h"

// Per-joint angle and homing state — kept here, not in the struct
static float  joint_angle[NUM_JOINTS]  = {0.0f, 0.0f, 0.0f};
static bool   joint_homed[NUM_JOINTS]  = {false, false, false};


//NOTES FOR DEBUGGING. CHECK TO MAKE SURE THAT FIRST ODRIVE IS THE NODE 0 AND WITH RESISTOR BRIDGED
Joint joints[NUM_JOINTS] = {
    { &odrv0, &odrv0_user_data, 0, 5.0f, 0.0f,  "ROTATE"    },
    { &odrv1, &odrv1_user_data, 1, 5.0f, 0.0f,  "LIFT"   },
    { &odrv2, &odrv2_user_data, 2, 5.0f, 0.0f,  "REACH"  },
};

void initJoints() {
    Serial.println("Joints initialized");
}

void readJointAngles() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        tcaSelect(joints[i].sensor_channel);
        delayMicroseconds(200);
        uint16_t raw       = readRawAS5600();
        joint_angle[i]     = computeAngle(i, raw);
    }
}

float getJointAngle(uint8_t id) {
    return joint_angle[id];
}

bool isJointHomed(uint8_t id) {
    return joint_homed[id];
}

void printJointStatus() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(joints[i].label);
        Serial.print("  angle=");
        Serial.print(joint_angle[i], 1);
        Serial.print("  homed=");
        Serial.println(joint_homed[i] ? "YES" : "NO");
    }
}