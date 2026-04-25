#include "states.h"
#include "odrive.h"
#include <cmath>
#include "LEDs.h"
#include "comms.h"
#include "buttons.h"
#include "USB.h"
#include "Wire.h"
#include "joint.h"
#include <cstddef>


extern Joint joints[NUM_JOINTS];
#include "admittance_controller.h"
#include "errors.h"

// Test function: command a fixed velocity to a joint for debugging
void testSetJointVelocity(int jointIdx, float velocity) {
    if (jointIdx < 0 || jointIdx >= NUM_JOINTS) {
        SerialUSB1.print("[TEST] Invalid joint index: ");
        SerialUSB1.println(jointIdx);
        return;
    }
    if (joints[jointIdx].odrive == nullptr) {
        SerialUSB1.print("[TEST] Joint ");
        SerialUSB1.print(jointIdx);
        SerialUSB1.println(" has no ODrive attached.");
        return;
    }
    SerialUSB1.print("[TEST] Setting joint ");
    SerialUSB1.print(jointIdx);
    SerialUSB1.print(" velocity to ");
    SerialUSB1.println(velocity, 4);
    joints[jointIdx].odrive->setVelocity(velocity, 0.0f);
}


state_e currState = BOOTUP;
errorCode_e currError = NO_ERROR;

// Telemetry pacing to avoid saturating the USB receive queue on host.
static const uint32_t JOINT_TELEM_INTERVAL_MS = 5;
static const uint32_t STATUS_TELEM_INTERVAL_MS = 100; // 10 Hz

// ── READY sub-state — file-scope so variables survive state re-entries ────────
// Using file-scope (not static locals) means resetReadyState() can zero them on
// every entry, preventing stale values if the system exits and re-enters READY.
enum ReadySubState { ADMITTANCE, MOVING_TO_HOME, RESTORING };
static ReadySubState readySubState          = ADMITTANCE;
static bool          readyTelemInit         = false;
static bool          enteredReady           = false;
static uint32_t      lastJointTelemMs       = 0;
static uint32_t      lastStatusTelemMs      = 0;
static uint32_t      lastAdmittanceUs       = 0;
static uint32_t      lastOdriveErrorCheckMs = 0;
static uint32_t      moveToHomeStartMs      = 0;
static uint32_t      lastHomingLogMs        = 0;
static int           homingJointIdx         = 0;  // which joint is currently being homed

static const float    HOME_TOL                    = 0.02f;
static const float    HOMING_VEL                  = 0.8f;   // t/s max approach speed
// Proportional gain for homing velocity: vel = clamp(HOMING_K * err, ±HOMING_VEL).
// At 0.4 turns error → full speed. At 0.02 turns → 0.04 t/s (soft landing).
// Direction is always derived from sign(err) — no per-joint home_vel_dir needed.
static const float    HOMING_K                    = 2.0f;
static const uint32_t MOVE_TO_HOME_TIMEOUT_MS     = 15000;
static const uint32_t ODRIVE_ERROR_CHECK_INTERVAL_MS = 10000;
static const uint32_t HOMING_LOG_INTERVAL_MS      = 500;

static void resetReadyState() {
    readySubState          = ADMITTANCE;
    readyTelemInit         = false;
    enteredReady           = false;
    lastJointTelemMs       = 0;
    lastStatusTelemMs      = 0;
    lastAdmittanceUs       = 0;
    lastOdriveErrorCheckMs = 0;
    moveToHomeStartMs      = 0;
    lastHomingLogMs        = 0;
    homingJointIdx         = 0;
    moveToHomePending      = false;  // discard any command that arrived before READY was entered
}

// ── HOMING timeout ────────────────────────────────────────────────────────────
static uint32_t       homingEnteredMs    = 0;
static bool           homingTimerStarted = false;
static const uint32_t HOMING_TIMEOUT_MS = 300000UL; // 5-minute operator timeout

static void resetHomingState() {
    homingEnteredMs    = 0;
    homingTimerStarted = false;
}

// Prepares one ODrive joint for velocity-control homing.
// Stays in velocity mode (no mode switch) — raises gains for the homing move and
// zeroes velocity so there is no current commanded until the loop starts.
static void setupJointForHoming(int i) {
    pumpEvents(can_intf);
    bool inClosedLoop = joints[i].user_data->received_heartbeat &&
                        joints[i].user_data->last_heartbeat.Axis_State == 8;
    if (!inClosedLoop) {
        SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
        SerialUSB1.println(" disarmed — re-arming...");
        joints[i].odrive->clearErrors();
        delay(30); pumpEvents(can_intf);

        // Apply soft gains BEFORE setState so the drive never briefly runs
        // with the high gains stored in flash (those are what cause the
        // encoder error that immediately disarms it again).
        joints[i].odrive->setVelGains(joints[i].home_vel_gain, joints[i].home_vel_int_gain);
        joints[i].odrive->setLimits(2.0f, 15.0f);
        pumpEvents(can_intf);

        joints[i].odrive->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        for (int k = 0; k < 20; k++) { delay(10); pumpEvents(can_intf); }
        inClosedLoop = joints[i].user_data->received_heartbeat &&
                       joints[i].user_data->last_heartbeat.Axis_State == 8;
        SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
        // Do not return on failure — fall through and attempt to set velocity
        // to 0; the MOVING_TO_HOME loop will retry next cycle if still disarmed.
        SerialUSB1.println(inClosedLoop ? " re-arm OK" : " re-arm FAILED — will retry next cycle");
    }
    // Raise gains for homing approach; RESTORING phase calls enable_velocity_control()
    // which resets them back to admittance values (vel_gain=0.001).
    joints[i].odrive->setVelGains(joints[i].home_vel_gain, joints[i].home_vel_int_gain);
    joints[i].odrive->setLimits(2.0f, 15.0f);
    joints[i].odrive->setVelocity(0.0f, 0.0f);
    pumpEvents(can_intf);
    for (int k = 0; k < 5; k++) { delay(10); pumpEvents(can_intf); }
    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
    SerialUSB1.print(" → vel homing  vel_gain=");
    SerialUSB1.print(joints[i].home_vel_gain, 3);
    SerialUSB1.print("  vel_int=");
    SerialUSB1.print(joints[i].home_vel_int_gain, 3);
    SerialUSB1.println("  [OK]");
}

/*
 * Verifies ODrive system is initialized and all ODrives are online.
 * Initializes all ODrive instances and waits for heartbeats via initMultiOdrives().
 * Communication buses are initialized once during startup in setup().
 * Returns true if all ODrives are online, false otherwise.
 */
bool verifyODrive(){
    if (!initMultiOdrives()) {
        setError(ODRIVE_ERROR);
        return false;
    }

    bool odrivesHealthy = true;
    odrivesHealthy &= printOdriveError(&odrv0, 0);
    odrivesHealthy &= printOdriveError(&odrv1, 1);
    odrivesHealthy &= printOdriveError(&odrv2, 2);

    if (!odrivesHealthy) {
        setError(ODRIVE_ERROR);
        return false;
    }

    return true;
}

/*
 * Verifies I2C setup and external encoder presence.
 * Checks mux ACK and ensures the expected number of active external encoder
 * channels return valid AS5600 readings.
 */
bool verifyI2C(){
    Wire.beginTransmission(TCA_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        SerialUSB1.print("[I2C][BOOTUP] TCA9548A ACK failed at 0x");
        SerialUSB1.print(TCA_ADDR, HEX);
        SerialUSB1.print(" wireErr=");
        SerialUSB1.println((int)error);
        setError(I2C_ERROR);
        return false;
    }

    uint8_t expectedExternalEncoders = 0;
    uint8_t detectedExternalEncoders = 0;

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);
        if (j == nullptr) {
            SerialUSB1.print("[I2C][BOOTUP] Null joint pointer at index ");
            SerialUSB1.println((int)i);
            setError(I2C_ERROR);
            return false;
        }

        if (j->use_onboard_encoder || j->sensor_channel == INACTIVE_CHANNEL) {
            continue;
        }

        expectedExternalEncoders++;

        tcaSelect(j->sensor_channel);
        uint16_t raw = readRawAS5600();
        if (raw == 0xFFFF) {
            SerialUSB1.print("[I2C][BOOTUP] AS5600 read failed on mux channel ");
            SerialUSB1.print((int)j->sensor_channel);
            SerialUSB1.print(" (joint index ");
            SerialUSB1.print((int)i);
            SerialUSB1.println(")");
            setError(I2C_ERROR);
            return false;
        }

        detectedExternalEncoders++;
    }

    if (detectedExternalEncoders != expectedExternalEncoders) {
        SerialUSB1.print("[I2C][BOOTUP] Encoder count mismatch expected=");
        SerialUSB1.print((int)expectedExternalEncoders);
        SerialUSB1.print(" detected=");
        SerialUSB1.println((int)detectedExternalEncoders);
        setError(I2C_ERROR);
        return false;
    }

    return true;
}

/*
 * Verifies all LED pins are initialized and responding correctly.
 * Calls Led::setup() to configure all pins as OUTPUT, then writes HIGH
 * to each LED and reads back to confirm the pin responded.
 * Returns true if all three LEDs respond correctly, false if any fail.
 */
bool verifyLED(){
    Led::setup();
    ONToggleLED(&Led::powerLed);
    if (digitalRead(Led::powerLed.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    ONToggleLED(&Led::dataLed);
    if (digitalRead(Led::dataLed.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    ONToggleLED(&Led::errorLed);
    if (digitalRead(Led::errorLed.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }

    return true;
}

/*
 * Verifies USB serial connection is open and host is actively connected.
 * Returns true if a ping command has been received from the host,
 * indicating the host PC is actively communicating with the device.
 */
bool verifyUSB(){
    return pingReceived;
}

// Checks whether the ODrive comms are online
bool verifyODriveComms(){
    if (
        !odrv0_user_data.received_heartbeat ||
        !odrv1_user_data.received_heartbeat ||
        !odrv2_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        return false;
    }
    return true;
}

/*
 * Convenience wrapper that checks all three connection layers are healthy.
 * Checks in order: USB → ODrive CAN → I2C encoders
 * Sets specific error code for whichever layer fails first.
 * Returns true only if all three layers pass, false if any fail.
 */
bool allConnectionsReady()
{
    if (!verifyUSB()) {
        setError(CONNECTION_ERROR);
        return false;
    }
    if (!verifyODriveComms()) {
        setError(ODRIVE_ERROR);
        return false;
    }
    if (!verifyI2C()) {
        setError(I2C_ERROR);
        return false;
    }
    return true;
}

/*
 * Check if a CMD_PING packet has been received.
 * Returns true if ping was received since last check.
 */
bool pollCmdPing() {
    if (pingEventPending) {
        pingEventPending = false;
        return true;
    }
    return false;
}

/*
 * Streams the latest joint angle/velocity snapshot to host PC as
 * a TELEM_JOINT_DATA packet.
 *
 * Sensing is performed once per READY cycle before control, so this
 * function only validates and serializes the already-updated joint state.
 * If any encoder fails during reading, sends ERROR_MESSAGE to host
 * and transitions to ERROR_STATE immediately.
 * Called every loop cycle in READY state.
 */
void enableI2CPacketSend()
{
    telemJointDataPayload data;

    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);

        if (j == nullptr) {
            setError(I2C_ERROR);
            sendErrorMessage("I2C error: invalid joint pointer");
            currState = ERROR_STATE;
            return;
        }

        if (!j->use_onboard_encoder && j->rawValue == 0xFFFF) {
            setError(I2C_ERROR);
            sendErrorMessage("I2C error: external encoder read failed");
            currState = ERROR_STATE;
            return;
        }

        buildTelemJointPayload(data, i, j->angle, j->velocity);
    }
    sendTelemJointData(data);
}

/*
 * Checks ODrive heartbeats are still active and streams ODrive
 * status to host PC as a TELEM_STATUS packet.
 * If any ODrive stops sending heartbeats, sends ERROR_MESSAGE
 * to host and transitions to ERROR_STATE immediately.
 */
void enableODrivePacketSend()
{
    telemStatusPayload status{};

    if (
        !odrv0_user_data.received_heartbeat ||
        !odrv1_user_data.received_heartbeat ||
        !odrv2_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        packet errPacket(ERROR_MESSAGE);
        std::vector<uint8_t> errBuffer = errPacket.serialize();
        Serial.write(errBuffer.data(), errBuffer.size());
        currState = ERROR_STATE;
        return;
    }

    // TODO: confirm what values armStatus and odriveFaults should hold
    // status.armStatus =
    // status.odriveFaults =

    sendTelemStatus(status);
}

/*
 * Commands all ODrive controllers to idle state before power is cut.
 */
void stopODrives(){
    emergencyStop();
}

/*
 * Turns off all LED indicators during shutdown sequence.
 */
void powerOffPeripherals(){
    OFFToggleLED(&Led::powerLed);
    OFFToggleLED(&Led::dataLed);
    OFFToggleLED(&Led::errorLed);
}

/*
 * Turns on the error LED to visually alert the operator of a fault.
 */
void turnOnErrorLED(){
    ONToggleLED(&Led::errorLed);
}

/*
 * Resets system state after operator acknowledges and resolves the fault.
 * Clears the error flag, turns off error LED, and returns to BOOTUP
 * to rerun all hardware verification before resuming operation.
 */
void errorRecovery(){
    clearError();
    OFFToggleLED(&Led::errorLed);
    resetReadyState();
    resetHomingState();
    currState = BOOTUP;
}

/*
 * Stores the current error code when a fault is detected.
 */
void setError(errorCode_e err) {
    currError = err;
}

/*
 * Returns the current error code.
 */
errorCode_e getError() {
    return currError;
}

/*
 * Resets the error code back to NO_ERROR.
 */
void clearError(){
    currError = NO_ERROR;
}

/*
 * Returns true if any error is currently active, false if system is healthy.
 */
bool errorCheck(){
    return currError != NO_ERROR;
}

/*
 * Transmits a specific error message over Serial based on the current error code.
 */
void sendStateErrorLog() {
    switch (currError)
    {
    case ODRIVE_ERROR:
        SerialUSB1.println("ERROR: ODRIVE FAILURE");
        break;
    case I2C_ERROR:
        SerialUSB1.println("ERROR: I2C FAILURE");
        break;
    case LED_ERROR:
        SerialUSB1.println("ERROR: LED FAILURE");
        break;
    case CONNECTION_ERROR:
        SerialUSB1.println("ERROR: CONNECTION FAILURE");
        break;
    case NO_ERROR:
        break;
    default:
        SerialUSB1.println("ERROR: UNKNOWN FAILURE");
        break;
    }
}

// Main State Machine
/*
 * Transitions:
 *   BOOTUP      --> IDLE        (all hardware verified)
 *   BOOTUP      --> ERROR_STATE (any hardware verification fails)
 *   IDLE        --> CONNECTED   (CMD_PING received from host)
 *   CONNECTED   --> ERROR_STATE (any connection check fails)
 *   CONNECTED   --> READY       (connections healthy, arm already homed)
 *   CONNECTED   --> HOMING      (connections healthy, arm not homed)
 *   HOMING      --> READY       (homing sequence complete)
 *   READY       --> POWERINGOFF (shutdown commanded)
 *   Any state   --> ERROR_STATE (fault detected)
 *   ERROR_STATE --> ERROR_STATE (latched fault until device reset)
 */
void stateUpdate()
{
  switch (currState) {
    // Verify all hardware before allowing any operation.
    // Success: power LED on, transition to IDLE.
    // Failure: transition to ERROR_STATE.
    case BOOTUP:
        if (verifyODrive() && verifyI2C() && verifyLED()) {
            ONToggleLED(&Led::powerLed);   // power LED on = system alive
            OFFToggleLED(&Led::errorLed);  // ensure error LED is off
            currState = IDLE;
        }
        else {
            currState = ERROR_STATE;
        }
        break;

    // Wait for CMD_PING from host PC.
    // PING received: transition to CONNECTED. (Pong is sent in handlePing)
    case IDLE:
        if (pollCmdPing()) {
            currState = CONNECTED;
        }
        break;

    // Verify all communication links are healthy.
    // Failure: transition to ERROR_STATE.
    // Success + already homed: transition to READY.
    // Success + not homed: transition to HOMING.
    case CONNECTED:
        if (!allConnectionsReady()) {
            currState = ERROR_STATE;
            break;
        }
        else if (isHomed()) {
            currState = READY;
        }
        else {
            currState = HOMING;
        }
        break;

    // Manual homing: arm is compliant (velocity mode, soft gains).
    // Operator physically places the arm at the home pose, then sends
    // CMD_CONFIRM_HOME from Unity. A 5-minute timeout transitions to ERROR_STATE
    // if the operator does not confirm, preventing indefinite powered standby.
    case HOMING:
    {
        if (!homingTimerStarted) {
            homingEnteredMs    = millis();
            homingTimerStarted = true;
            SerialUSB1.println("[HOMING] Awaiting CMD_CONFIRM_HOME from operator (5-min timeout).");
        }

        if (confirmHomePending) {
            confirmHomePending = false;
            moveToHomePending  = false;  // prevent READY from triggering a second homing sequence
            confirmHome();
            resetHomingState();
            currState = READY;
            break;
        }

        if ((millis() - homingEnteredMs) >= HOMING_TIMEOUT_MS) {
            SerialUSB1.println("[HOMING] Operator timeout — entering ERROR_STATE.");
            setError(CONNECTION_ERROR);
            resetHomingState();
            currState = ERROR_STATE;
        }
        break;
    }

    // Normal operating state — admittance control always running.
    // data LED turns on once on entry via static flag.
    // Sub-states: ADMITTANCE (normal), MOVING_TO_HOME (position control), RESTORING (back to velocity).
    case READY:
    {
        // All sub-state variables are file-scope (declared above stateUpdate).
        // resetReadyState() is called from errorRecovery() to ensure a clean
        // entry every time READY is reached, even after error recovery.

        if (!enteredReady) {
            ONToggleLED(&Led::dataLed);
            resetAdmittanceController();
            lastAdmittanceUs = micros();
            enteredReady = true;
        }

        if (!readyTelemInit) {
            uint32_t now = millis();
            lastJointTelemMs = now;
            lastStatusTelemMs = now;
            readyTelemInit = true;
        }

        // --- MOVING_TO_HOME entry: zero velocities and set up first joint ---
        if (moveToHomePending && readySubState == ADMITTANCE) {
            moveToHomePending = false;
            SerialUSB1.println("[HOMING] Starting sequential homing.");

            // Flush any residual admittance velocity commands before switching modes.
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (!joints[i].use_onboard_encoder || joints[i].odrive == nullptr) continue;
                joints[i].odrive->setVelocity(0.0f, 0.0f);
            }
            delay(50); pumpEvents(can_intf);

            // Find the first ODrive joint to home.
            homingJointIdx = 0;
            while (homingJointIdx < NUM_JOINTS &&
                   (!joints[homingJointIdx].use_onboard_encoder ||
                    joints[homingJointIdx].odrive == nullptr))
                homingJointIdx++;

            if (homingJointIdx >= NUM_JOINTS) {
                SerialUSB1.println("[HOMING] No ODrive joints — skipping.");
                readySubState = RESTORING;
            } else {
                setupJointForHoming(homingJointIdx);
                moveToHomeStartMs = millis();
                readySubState = MOVING_TO_HOME;
            }
        }

        // --- MOVING_TO_HOME loop: velocity-control homing, one joint at a time ---
        if (readySubState == MOVING_TO_HOME) {
            int i = homingJointIdx;
            uint32_t nowMs = millis();
            bool doLog = (nowMs - lastHomingLogMs) >= HOMING_LOG_INTERVAL_MS;
            bool advanceJoint = false;

            // If the drive disarmed mid-move, attempt one recovery then continue.
            bool armed = joints[i].user_data->received_heartbeat &&
                         joints[i].user_data->last_heartbeat.Axis_State == 8;
            if (!armed) {
                SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                SerialUSB1.println(" disarmed mid-move — recovering...");
                joints[i].odrive->clearErrors();
                delay(20); pumpEvents(can_intf);

                // Apply soft homing gains BEFORE entering closed loop so the drive
                // never runs with the high flash gains, which is what trips the
                // encoder/velocity error in the first place.
                joints[i].odrive->setVelGains(joints[i].home_vel_gain, joints[i].home_vel_int_gain);
                joints[i].odrive->setLimits(2.0f, 15.0f);
                pumpEvents(can_intf);

                joints[i].odrive->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                for (int k = 0; k < 20; k++) { delay(10); pumpEvents(can_intf); }

                bool reArmed = joints[i].user_data->received_heartbeat &&
                               joints[i].user_data->last_heartbeat.Axis_State == 8;
                SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                // Log outcome but do NOT skip — let the homing loop retry on the
                // next cycle rather than permanently abandoning this joint.
                SerialUSB1.println(reArmed ? " re-armed OK" : " re-arm FAILED — will retry");
            }

            if (!advanceJoint) {
                float pos = joints[i].user_data->last_feedback.Pos_Estimate;
                float err = joints[i].home_pos - pos;

                // Proportional approach: speed scales with distance, direction from
                // sign(err). This means the joint always moves toward home regardless
                // of which side it starts on, and decelerates naturally as it closes in.
                // At |err| >= 0.4 turns → full HOMING_VEL. At HOME_TOL → ~0.04 t/s.
                float vel_cmd = constrain(HOMING_K * err, -HOMING_VEL, HOMING_VEL);
                if (fabsf(err) <= HOME_TOL) vel_cmd = 0.0f;
                joints[i].odrive->setVelocity(vel_cmd, 0.0f);
                pumpEvents(can_intf);

                if (doLog) {
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.print("  pos="); SerialUSB1.print(pos, 4);
                    SerialUSB1.print("  err="); SerialUSB1.print(err, 4);
                    SerialUSB1.print("  vel="); SerialUSB1.print(vel_cmd, 2);
                    SerialUSB1.println(fabsf(err) <= HOME_TOL ? "  [AT HOME]" : "  [moving]");
                }

                if (fabsf(err) <= HOME_TOL) {
                    joints[i].odrive->setVelocity(0.0f, 0.0f);
                    pumpEvents(can_intf);
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.println(" reached home.");
                    advanceJoint = true;
                } else if ((nowMs - moveToHomeStartMs) > MOVE_TO_HOME_TIMEOUT_MS) {
                    joints[i].odrive->setVelocity(0.0f, 0.0f);
                    pumpEvents(can_intf);
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.println(" timed out.");
                    advanceJoint = true;
                }
            }

            if (doLog) lastHomingLogMs = nowMs;

            if (advanceJoint) {
                // Advance to the next ODrive joint.
                do { homingJointIdx++; }
                while (homingJointIdx < NUM_JOINTS &&
                       (!joints[homingJointIdx].use_onboard_encoder ||
                        joints[homingJointIdx].odrive == nullptr));

                if (homingJointIdx >= NUM_JOINTS) {
                    SerialUSB1.println("[HOMING] All joints homed.");
                    readySubState = RESTORING;
                } else {
                    setupJointForHoming(homingJointIdx);
                    moveToHomeStartMs = millis();
                }
            }
        }

        // --- RESTORING: switch back to velocity/admittance mode ---
        if (readySubState == RESTORING) {
            SerialUSB1.println("[READY] Restoring velocity (admittance) mode.");
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (joints[i].use_onboard_encoder && joints[i].odrive != nullptr) {
                    enable_velocity_control(*joints[i].odrive, *joints[i].user_data, i);
                }
            }
            resetAdmittanceController();
            lastAdmittanceUs = micros();
            readySubState = ADMITTANCE;
        }

        // Normal admittance loop — only runs when not in a move-to-home sequence.
        if (readySubState == ADMITTANCE) {
            readJointAngles();
            uint32_t nowUs = micros();
            float dt = (nowUs - lastAdmittanceUs) * 1e-6f;
            lastAdmittanceUs = nowUs;
            stepAdmittanceController(dt);
        }

        uint32_t now = millis();
        if ((now - lastJointTelemMs) >= JOINT_TELEM_INTERVAL_MS) {
            enableI2CPacketSend();
            lastJointTelemMs = now;
        }

        if ((now - lastStatusTelemMs) >= STATUS_TELEM_INTERVAL_MS) {
            enableODrivePacketSend();
            lastStatusTelemMs = now;
        }

        // Periodically read ODrive error registers so faults show up in serial.
        if ((now - lastOdriveErrorCheckMs) >= ODRIVE_ERROR_CHECK_INTERVAL_MS) {
            printOdriveError(&odrv0, 0);
            printOdriveError(&odrv1, 1);
            printOdriveError(&odrv2, 2);
            lastOdriveErrorCheckMs = now;
        }
        break;
    }

    // Controlled shutdown sequence.
    // Motors idled gracefully before power is cut.
    case POWERINGOFF:
        stopODrives();
        powerOffPeripherals();
        break;

    // Safe all hardware immediately and report fault.
    // Fault is latched to avoid auto-restart loops and noisy serial output.
    case ERROR_STATE:
    {
        static bool errorLatched = false;
        if (!errorLatched) {
            stopODrives();
            turnOnErrorLED();
            sendStateErrorLog();
            errorLatched = true;
        }
        break;
    }

    // Unknown or corrupted state — should never be reached.
    default:
        stopODrives();
        turnOnErrorLED();
        sendStateErrorLog();
        break;
    }
}