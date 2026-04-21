#include "joint.h"
#include "comms.h"
#include "odrive.h"

static bool system_homed = false;



/*
joint # |  Odrv/enc
    0   |   odrv 0
    1   |   odrv 1
    2   |   odrv 2
    3   |   enc 3
    4   |   enc 2
    5   |   enc 1
    6   |   enc 0

*/
Joint joints[NUM_JOINTS] = {
    // odrive  user_data         sensor_ch         max_t  home   onboard  label      angle  raw  vel    homed  target_t
    { &odrv0,  &odrv0_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "ROTATE",  0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv1,  &odrv1_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "REACH",   0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv2,  &odrv2_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    "LIFT",    0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          3,                0.0f,  0.0f,  false,   "EXT_CH3", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          2,                0.0f,  0.0f,  false,   "EXT_CH2", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          1,                0.0f,  0.0f,  false,   "EXT_CH1", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          0,                0.0f,  0.0f,  false,   "EXT_CH0", 0.0f,  0,   0.0f,  false, 0.0f },

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

#ifndef ODRIVE_FULL
    // Disable joint 0 (ROTATE / odrv0) — not connected in test setup.
    // Admittance controller already skips joints where odrive == nullptr.
    joints[0].odrive = nullptr;
    SerialUSB1.println("[CONFIG] ODRIVE_TEST_1_2: joint 0 (ROTATE) disabled");
#endif

    SerialUSB1.println("Joints initialized with configured mapping");
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
            SerialUSB1.print("[WARN] AS5600 read failed on channel ");
            SerialUSB1.println((int)joints[i].sensor_channel);
            joints[i].velocity = 0.0f;
            }
        }
    }
}


// Print joint status to serial with nice formatting
void printJointStatus() {
    // Header
    SerialUSB1.println();
    SerialUSB1.println("======================================== JOINT STATUS ========================================");
    SerialUSB1.println(" ID | Label         | Angle      | Sensor        | Velocity  | Homed | Torque");
    SerialUSB1.println("----+---------------+------------+---------------+-----------+-------+----------");
    
    // Joint data
    for (int i = 0; i < NUM_JOINTS; i++) {
        // Joint ID
        SerialUSB1.print("  ");
        SerialUSB1.print(i);
        SerialUSB1.print(" | ");
        
        // Label (left-aligned, 13 chars)
        const char* label = (joints[i].label != nullptr) ? joints[i].label : "UNNAMED";
        SerialUSB1.print(label);
        for (int pad = 0; pad < (13 - strlen(label)); pad++) SerialUSB1.print(" ");
        SerialUSB1.print(" | ");
        
        // Angle (right-aligned, 10 chars with one decimal)
        SerialUSB1.print("     ");
        SerialUSB1.print(joints[i].angle, 1);
        SerialUSB1.print("° | ");
        
        // Sensor channel/type (left-aligned, 13 chars)
        if (joints[i].sensor_channel == INACTIVE_CHANNEL) {
            SerialUSB1.print("ODrive      ");
        } else {
            SerialUSB1.print("CH");
            SerialUSB1.print(joints[i].sensor_channel);
            for (int pad = 0; pad < (11 - (joints[i].sensor_channel >= 10 ? 2 : 1)); pad++) SerialUSB1.print(" ");
        }
        SerialUSB1.print(" | ");
        
        // Velocity (right-aligned, 9 chars)
        SerialUSB1.print("   ");
        SerialUSB1.print(joints[i].velocity, 2);
        SerialUSB1.print(" | ");
        
        // Homed status
        SerialUSB1.print(joints[i].is_homed ? "YES  " : "NO   ");
        SerialUSB1.print("| ");
        
        // Torque
        SerialUSB1.print(joints[i].target_torque, 2);
        SerialUSB1.println(" Nm");
    }
    SerialUSB1.println("==========================================================================================");
    SerialUSB1.println();
}

/*
 * Command the arm to move to the home position.
 * Sends homing command to all joints once via static flag.
 */
/*
 * Commands all un-homed joints that use onboard encoders to move to
 * their configured home position via ODrive position control.
 * Sets is_homed = true per joint once the command is sent.
 * system_homed is set true only when ALL joints are homed.
 * Safe to call repeatedly — already-homed joints are skipped.
 */
void startHoming() {
    bool allHomed = true;

    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].use_onboard_encoder) {
            if (!joints[i].is_homed) {
                float homePos = joints[i].home_pos;
                joints[i].odrive->setPosition(homePos);
                joints[i].is_homed = true;  // mark homed after command sent
            }
            // if still not homed for any reason, not all joints done
            if (!joints[i].is_homed) {
                allHomed = false;
            }
        }
    }

    system_homed = allHomed;
}

/*
 * Check if the arm has been homed (home position established).
 */
bool isHomed() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].use_onboard_encoder) {
            if (!joints[i].is_homed) {
                return false;  // at least one joint still needs homing
            }
        }
    }
    return system_homed;  // all onboard encoder joints are homed
}

/*
 * Polls whether the homing sequence is complete by checking two conditions:
 *   1. system_homed flag is set (all homing commands have been sent)
 *   2. All homed joints are within position tolerance of their home_pos
 *      according to the latest ODrive position feedback.
 * Returns true only when all joints have reached home within tolerance.
 */
bool verifyHoming() {
    if (!system_homed) {
        return false;
    }

    const float HOME_TOLERANCE_TURNS = 0.01f;  // adjust to your mechanical tolerance

    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].use_onboard_encoder && joints[i].is_homed) {
            float currentPos = joints[i].user_data->last_feedback.Pos_Estimate;
            float homePos    = joints[i].home_pos;
            float err        = currentPos - homePos;
            if (err < 0) err = -err;  // abs without including <cmath>

            if (err > HOME_TOLERANCE_TURNS) {
                return false;  // this joint hasn't settled yet
            }
        }
    }

    return true;  // all joints within tolerance
}