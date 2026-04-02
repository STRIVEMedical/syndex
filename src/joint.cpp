#include "joint.h"
#include "comms.h"
#include "odrive.h"

Joint joints[NUM_JOINTS] = {
    // odrive  user_data         sensor_ch         max_t  home   onboard  label      angle  raw  vel    homed  target_t
    { nullptr, nullptr,          0,                0.0f,  0.0f,  false,   "EXT_CH0", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          1,                0.0f,  0.0f,  false,   "EXT_CH1", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          2,                0.0f,  0.0f,  false,   "EXT_CH2", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          3,                0.0f,  0.0f,  false,   "EXT_CH3", 0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv0,  &odrv0_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "ROTATE",  0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv1,  &odrv1_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "REACH",   0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv2,  &odrv2_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "LIFT",    0.0f,  0,   0.0f,  false, 0.0f },
};

void initJoints() {
    // Keep hardware mapping and labels; reset only runtime state.
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].angle = 0.0f;
        joints[i].rawValue = 0;
        joints[i].velocity = 0.0f;
        joints[i].is_homed = false;
        joints[i].target_torque = 0.0f;
    }

    Serial.println("Joints initialized with configured mapping");
}

// Get joint pointer by ID
Joint* getJoint(uint8_t id) {
    if (id >= NUM_JOINTS){
        return nullptr;
    }

    return &joints[id];
}


// Read from all odrives and external encoders (also reads and updated velocity)
void readJointAngles(){
    for(int i = 0; i < NUM_JOINTS; i++){
        //Joint reads from Odrive encoder
        if(joints[i].use_onboard_encoder){
            //read position from CAN callback
            float turns = joints[i].user_data->last_feedback.Pos_Estimate;
            float velocity_est = joints[i].user_data->last_feedback.Vel_Estimate;
            joints[i].angle = turns * 360.0f;
            joints[i].velocity = velocity_est;

        }else{
            //Joint reads from external encoder
            tcaSelect(joints[i].sensor_channel);
            delayMicroseconds(200);
            uint16_t raw  = readRawAS5600();
            joints[i].rawValue = raw;
            if (raw != 0xFFFF) {
                joints[i].angle = computeAngle(i, raw);
                joints[i].velocity = 0.0f;
            } else {
            Serial.print("[WARN] AS5600 read failed on channel ");
            Serial.println((int)joints[i].sensor_channel);
            joints[i].velocity = 0.0f;
            }
        }
    }
}


// Print joint status to serial
void printJointStatus() {
    Serial.println("=== Joint Status ===");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print("J");
        Serial.print(i);
        Serial.print(": ");
        if (joints[i].label != nullptr) {
            Serial.print(joints[i].label);
        } else {
            Serial.print("UNNAMED");
        }
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