#include "joint.h"
#include "i2c.h"
#include "main.h"

// Per-joint angle and homing state — kept here, not in the struct
static float joint_angle[NUM_JOINTS] = {0};
static bool   joint_homed[NUM_JOINTS]  = {false};

static constexpr uint8_t INACTIVE_CHANNEL = 255;

//ODRIVE 0 READS FROM ONBOARD ENCODER

//NOTES FOR DEBUGGING. CHECK TO MAKE SURE THAT FIRST ODRIVE IS THE NODE 0 AND WITH RESISTOR BRIDGED
Joint joints[NUM_JOINTS] = {
    // odrive       user_data            ch    max_t  home   has_odrv  onboard_enc  label
    { nullptr, nullptr,                  0,    0.0f,  0.0f,  false,    false,       "EXT_CH0"     },
    { nullptr, nullptr,                  1,    0.0f,  0.0f,  false,    false,       "EXT_CH1"     },
    { nullptr, nullptr,                  2,    0.0f,  0.0f,  false,    false,       "EXT_CH2"     },
    { nullptr, nullptr,                  3,    0.0f,  0.0f,  false,    false,       "EXT_CH3"     },
    { &odrv0, &odrv0_user_data,          INACTIVE_CHANNEL,  5.0f,  0.0f,  true,     true,        "REACH"       },
    { nullptr, nullptr,                  INACTIVE_CHANNEL,  0.0f,  0.0f,  false,    false,       "UNUSED_5"    },
    { nullptr, nullptr,                  INACTIVE_CHANNEL,  0.0f,  0.0f,  false,    false,       "UNUSED_6"    },
};
void initJoints() {
    Serial.println("Joints initialized");
}

void readJointAngles() {
    for (int i = 0; i < NUM_JOINTS; i++) {

        if (joints[i].use_onboard_encoder) {
            // Pull position directly from ODrive encoder feedback over CAN
            // Pos_Estimate is in turns — convert to degrees
            if (joints[i].user_data->received_feedback) {
                float turns = joints[i].user_data->last_feedback.Pos_Estimate;
                joint_angle[i] = turns * 360.0f;
                DBG_FLT("[ENC] Joint 0 onboard pos (deg): ", joint_angle[i]);
            } else {
                DBG("[WARN] Joint 0 onboard encoder not yet received");
            }

        } else {
            if (joints[i].sensor_channel > 3) {
                continue;
            }

            // AS5600 via I2C mux for all joints configured to use external encoders
            tcaSelect(joints[i].sensor_channel);
            delayMicroseconds(200);
            uint16_t raw     = readRawAS5600();
            if (raw != 0xFFFF) {
                joint_angle[i] = computeAngle(i, raw);
            } else {
                DBG_VAL("[WARN] AS5600 read failed on channel ", joints[i].sensor_channel);
            }

        }
        // all external-encoder joints fall into the else branch above
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