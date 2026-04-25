#include "joint.h"
#include "comms.h"
#include "odrive.h"




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
// home_vel_gain / home_vel_int_gain are per-joint.
// Max first-cycle current ≈ vel_gain × vel_limit (vel_limit is 2.0 t/s during homing).
// Tuning rules:
//   Shakes or encoder error (disarm=0x1000) → halve vel_gain
//   Can't start moving against gravity       → integrator builds over ~5-10s; raise vel_int if needed
//   Overshoots / oscillates near home        → halve vel_int_gain
//
// Homing direction is computed automatically from sign(err) — no home_vel_dir needed.
// Field order: odrive, user_data, sensor_ch, max_torque, home_pos, use_onboard_encoder,
//              home_vel_gain, home_vel_int_gain, label, angle, rawValue, velocity, is_homed, target_torque
Joint joints[NUM_JOINTS] = {
    // odrive  user_data         sensor_ch         max_t  home   onboard  home_vg  home_vi  label      angle  raw  vel    homed  target_t
    { &odrv0,  &odrv0_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    0.01f,   0.005f,    "ROTATE",  0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv1,  &odrv1_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    0.167f,    0.333f,    "REACH",   0.0f,  0,   0.0f,  false, 0.0f },
    { &odrv2,  &odrv2_user_data, INACTIVE_CHANNEL, 5.0f,  0.0f,  true,    0.167f,    0.333f,    "LIFT",    0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          3,                0.0f,  0.0f,  false,   0.0f,    0.0f,    "EXT_CH3", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          2,                0.0f,  0.0f,  false,   0.0f,    0.0f,    "EXT_CH2", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          1,                0.0f,  0.0f,  false,   0.0f,    0.0f,    "EXT_CH1", 0.0f,  0,   0.0f,  false, 0.0f },
    { nullptr, nullptr,          0,                0.0f,  0.0f,  false,   0.0f,    0.0f,    "EXT_CH0", 0.0f,  0,   0.0f,  false, 0.0f },
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
        for (int pad = 0; pad < (13 - (int)strlen(label)); pad++) SerialUSB1.print(" ");
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
 * Returns true if all ODrive-backed joints have had their home reference
 * established via confirmHome(). Used by the CONNECTED state to decide
 * whether to enter HOMING on boot (always false after a power cycle).
 */
bool isHomed() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].use_onboard_encoder && !joints[i].is_homed) {
            return false;
        }
    }
    return true;
}


/*
 * Latches the current encoder position as zero on all active ODrives and external
 * encoders, then marks every ODrive-backed joint as homed.
 *
 * ODrive joints: each drive is idled before setAbsolutePosition() is called so
 * that the position step does not cause a velocity spike in the closed-loop
 * controller. Velocity control mode is set explicitly before re-entering
 * closed-loop, regardless of what is stored in ODrive flash.
 *
 * External encoder joints: zeroOffset[] and multi-turn counters are reset to
 * the current raw reading so computeAngle() returns 0 at the home pose.
 */
void confirmHome() {
    // ── Phase A: zero external encoder joints ────────────────────────────────
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].use_onboard_encoder || joints[i].sensor_channel == INACTIVE_CHANNEL) continue;
        tcaSelect(joints[i].sensor_channel);
        delayMicroseconds(200);
        uint16_t raw = readRawAS5600();
        if (raw != 0xFFFF) {
            uint8_t ch = joints[i].sensor_channel;
            zeroOffset[ch] = raw * 360.0f / 4096.0f;
            turns[ch]      = 0;
            lastRaw[ch]    = raw;
            SerialUSB1.print("[HOMING] External enc ch");
            SerialUSB1.print(ch);
            SerialUSB1.print(" zeroed raw=");
            SerialUSB1.println(raw);
        } else {
            SerialUSB1.print("[HOMING] WARNING: AS5600 read failed on ch");
            SerialUSB1.println(joints[i].sensor_channel);
        }
    }

    // Phase B: for each ODrive joint — idle → set position → re-arm in velocity mode
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (!joints[i].use_onboard_encoder || joints[i].odrive == nullptr) continue;

        // 1. idle the drive before touching pos_estimate.
        //    This prevents the velocity spike that setAbsolutePosition causes
        //    when called during closed-loop control.
        joints[i].odrive->setState(ODriveAxisState::AXIS_STATE_IDLE);
        delay(50);
        pumpEvents(can_intf);

        // 2. Set encoder reference to 0 while drive is idle (no control loop = no spike).
        joints[i].odrive->setAbsolutePosition(0.0f);
        delay(20);
        pumpEvents(can_intf);

        // 3. Set velocity control mode and gains BEFORE re-entering closed loop.
        //    This ensures the drive enters closed loop in the correct mode
        //    regardless of what is stored in ODrive flash.
        enable_velocity_control(*joints[i].odrive, *joints[i].user_data, i);

        // 4. Clear any errors accumulated during the idle/position-set sequence.
        joints[i].odrive->clearErrors();
        delay(20);
        pumpEvents(can_intf);

        // 5. Re-enter closed-loop control.
        joints[i].odrive->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(50);
        pumpEvents(can_intf);

        joints[i].is_homed = true;
        SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
        SerialUSB1.println(" homed and re-armed in velocity mode.");
    }

    SerialUSB1.println("[HOMING] All joints homed.");
}