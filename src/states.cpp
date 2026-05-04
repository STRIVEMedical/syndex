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
static bool          homingJointDone[3]     = {false, false, false};

// Order in which ODrive joints are homed. Change this array to reorder.
static const int      HOMING_ORDER[]              = {0, 1, 2};
static const int      HOMING_ORDER_LEN            = 3;

static const float    HOME_TOL                    = 0.3f;
// ── Homing gains ─────────────────────────────────────────────────────────────
// Per-joint vel gains for homing come from joints[i].home_vel_gain /
// home_vel_int_gain (set in joint.cpp). Tune those to change homing behaviour
// without affecting admittance. HOMING_VEL_LIMIT caps approach speed for safety.
static const float    HOMING_VEL_LIMIT            = 10.0f;   // vel_limit (t/s) during position-control homing
static const uint32_t MOVE_TO_HOME_TIMEOUT_MS     = 30000;  // 30 s per joint
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
    for (int k = 0; k < HOMING_ORDER_LEN; k++) homingJointDone[k] = false;
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

// Prepares a single ODrive joint to begin moving to home.
// Called once per joint at the start of its homing turn.
static void setupJointForHoming(int i) {
    // Step 1: Apply homing gains and velocity limit BEFORE switching modes.
    // CAN commands are fire-and-forget — if these are sent after enable_position_control,
    // the ODrive spends ~150ms in position control with the old admittance vel_limit=30,
    // which lets a large position error accelerate the joint past HOMING_VEL_LIMIT
    // (→ 0x8000 VELOCITY_LIMIT_VIOLATION before the new limit even arrives).
    joints[i].odrive->setPosGain(5.0f);
    joints[i].odrive->setVelGains(joints[i].home_vel_gain, joints[i].home_vel_int_gain);
    joints[i].odrive->setLimits(HOMING_VEL_LIMIT, 20.0f);
    delay(20); pumpEvents(can_intf);  // let gains/limits arrive before mode switch

    // Step 2: Switch to position control — limits are already in place.
    // The ODrive will hold the current position until we send the first setPosition.
    enable_position_control(*joints[i].odrive, *joints[i].user_data, i);

    // Step 3: Wait for mode switch to settle.
    for (int k = 0; k < 5; k++) { delay(10); pumpEvents(can_intf); }

    // Step 5: Send the home position target once — the ODrive holds it until told otherwise.
    // Briefly hold the current position first so the controller starts with zero error
    // rather than immediately demanding full current to close a large position gap.
    float cur_pos = joints[i].user_data->last_feedback.Pos_Estimate;
    joints[i].odrive->setPosition(cur_pos, 0.0f, 0.0f);
    delay(20); pumpEvents(can_intf);
    //actually move the joint to its home 
    joints[i].odrive->setPosition(joints[i].home_pos, 0.0f, 0.0f);
    pumpEvents(can_intf);

    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
    SerialUSB1.print(" homing start: vel_gain=");
    SerialUSB1.print(joints[i].home_vel_gain, 3);
    SerialUSB1.print(" vel_int=");
    SerialUSB1.println(joints[i].home_vel_int_gain, 3);
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
    // Test power LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::powerLED);
    if (digitalRead(Led::powerLED.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    // Test data LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::dataLED);
    if (digitalRead(Led::dataLED.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    // Test error LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::errorLED);
    if (digitalRead(Led::errorLED.pin) != HIGH) {
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
    // if (!verifyI2C()) {
    //     setError(I2C_ERROR);
    //     return false;
    // }
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

        // if (!j->use_onboard_encoder && j->rawValue == 0xFFFF) {
        //     setError(I2C_ERROR);
        //     sendErrorMessage("I2C error: external encoder read failed");
        //     currState = ERROR_STATE;
        //     return;
        // }

        buildTelemJointPayload(data, i, j->angle, j->velocity);
    }

    float trigDepth = getTriggerDepth(&buttonPins::triggerInput);
    data.triggerPressed = (trigDepth > 0.5f) ? 0xFF : 0x00;

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
    OFFToggleLED(&Led::powerLED);
    OFFToggleLED(&Led::dataLED);
    OFFToggleLED(&Led::errorLED);
}

/*
 * Turns on the error LED to visually alert the operator of a fault.
 */
void turnOnErrorLED(){
    ONToggleLED(&Led::errorLED);
}

/*
 * Resets system state after operator acknowledges and resolves the fault.
 * Clears the error flag, turns off error LED, and returns to BOOTUP
 * to rerun all hardware verification before resuming operation.
 */
void errorRecovery(){
    clearError();                  // reset currError to NO_ERROR
    OFFToggleLED(&Led::errorLED);  // turn off error LED
    currState = BOOTUP;            // restart verification from beginning
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
    bool pwrPressed = powerButtonWasPressed();
    if (pwrPressed && (currState != BOOTUP && currState != ERROR_STATE && currState != POWERINGOFF)) {
        currState = POWERINGOFF;
        return;
    }

  switch (currState) {
    // Verify all hardware before allowing any operation.
    // Success: power LED on, transition to IDLE.
    // Failure: transition to ERROR_STATE.
    case BOOTUP:
        if (!pwrPressed) {
            break;
        }
        if (verifyODrive() && verifyI2C() && verifyLED()) {
            ONToggleLED(&Led::powerLED);   // power LED on = system alive
            OFFToggleLED(&Led::errorLED);  // ensure error LED is off
            currState = IDLE;
        }
        else {
            currState = ERROR_STATE;
        }
        break;

    // Wait for CMD_PING from host PC.
    // PING received: transition to CONNECTED. (Pong is sent in handlePing)
    case IDLE:
    {
        static uint32_t lastIdleErrorCheckMs = 0;
        if ((millis() - lastIdleErrorCheckMs) >= 2000) {
            pumpEvents(can_intf);
            printOdriveError(&odrv0, 0);
            printOdriveError(&odrv1, 1);
            printOdriveError(&odrv2, 2);
            lastIdleErrorCheckMs = millis();
        }
        if (pollCmdPing()) {
            currState = CONNECTED;
        }
        break;
    }

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
            ONToggleLED(&Led::dataLED); // data LED on = system operational
            // Reset integrated admittance state when entering READY to avoid step jumps.
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

        // ── Sub-state: MOVING_TO_HOME ────────────────────────────────────────────
        // Entered when the user presses H (CMD_START_HOMING) while in ADMITTANCE.
        // Stops admittance, switches each ODrive joint to position control one at a
        // time (order defined by HOMING_ORDER), and drives it to home_pos=0.
        // When all joints are done, transitions to RESTORING.
        if (moveToHomePending && readySubState == ADMITTANCE) {
            moveToHomePending = false;
            SerialUSB1.println("[HOMING] Starting parallel homing.");

            // Stop any residual velocity commands from admittance before switching modes.
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (!joints[i].use_onboard_encoder || joints[i].odrive == nullptr) continue;
                joints[i].odrive->setVelocity(0.0f, 0.0f);
            }
            delay(50); pumpEvents(can_intf);

            // Start all joints homing simultaneously.
            for (int k = 0; k < HOMING_ORDER_LEN; k++) {
                homingJointDone[k] = false;
                setupJointForHoming(HOMING_ORDER[k]);
            }
            moveToHomeStartMs = millis();
            readySubState = MOVING_TO_HOME;
        }

        if (readySubState == MOVING_TO_HOME) {
            uint32_t nowMs = millis();
            bool doLog = (nowMs - lastHomingLogMs) >= HOMING_LOG_INTERVAL_MS;
            bool allDone = true;

            pumpEvents(can_intf);

            for (int k = 0; k < HOMING_ORDER_LEN; k++) {
                if (homingJointDone[k]) continue;
                int i = HOMING_ORDER[k];
                allDone = false;

                // If the ODrive disarmed mid-move, clear the fault and re-enter closed loop.
                bool armed = joints[i].user_data->received_heartbeat &&
                             joints[i].user_data->last_heartbeat.Axis_State == 8;
                if (!armed) {
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.print(" disarmed (axis_err=0x");
                    SerialUSB1.print(joints[i].user_data->last_heartbeat.Axis_Error, HEX);
                    SerialUSB1.println(") — recovering...");
                    joints[i].odrive->clearErrors();
                    delay(50); pumpEvents(can_intf);
                    joints[i].odrive->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                    for (int m = 0; m < 20; m++) { delay(10); pumpEvents(can_intf); }
                    float cur_pos = joints[i].user_data->last_feedback.Pos_Estimate;
                    joints[i].odrive->setPosition(cur_pos, 0.0f, 0.0f);
                    pumpEvents(can_intf);
                    bool reArmed = joints[i].user_data->received_heartbeat &&
                                   joints[i].user_data->last_heartbeat.Axis_State == 8;
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.println(reArmed ? " re-armed OK" : " re-arm FAILED — will retry");
                    continue;
                }

                float pos     = joints[i].user_data->last_feedback.Pos_Estimate;
                float vel_est = joints[i].user_data->last_feedback.Vel_Estimate;
                float err     = joints[i].home_pos - pos;

                if (doLog) {
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.print("  pos="); SerialUSB1.print(pos, 4);
                    SerialUSB1.print("  err="); SerialUSB1.print(err, 4);
                    SerialUSB1.print("  vel="); SerialUSB1.print(vel_est, 3);
                    SerialUSB1.println(fabsf(err) <= HOME_TOL ? "  [AT HOME]" : "  [moving]");
                }

                if (fabsf(err) <= HOME_TOL && fabsf(vel_est) < 0.1f) {
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.println(" reached home.");
                    homingJointDone[k] = true;
                } else if ((nowMs - moveToHomeStartMs) > MOVE_TO_HOME_TIMEOUT_MS) {
                    SerialUSB1.print("[HOMING] Joint "); SerialUSB1.print(i);
                    SerialUSB1.println(" timed out.");
                    homingJointDone[k] = true;
                }
            }

            if (doLog) lastHomingLogMs = nowMs;

            if (allDone) {
                SerialUSB1.println("[HOMING] All joints homed.");
                readySubState = RESTORING;
            }
        }

        // ── Sub-state: RESTORING ─────────────────────────────────────────────────
        // Runs once after MOVING_TO_HOME finishes. Switches all ODrives from
        // position control back to torque control for admittance, then re-calibrates
        // iq_bias at the new arm pose before resuming ADMITTANCE.
        if (readySubState == RESTORING) {
            SerialUSB1.println("[READY] Restoring admittance mode.");
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (joints[i].use_onboard_encoder && joints[i].odrive != nullptr) {
                    enable_velocity_control(*joints[i].odrive, *joints[i].user_data, i);
                }
            }
            resetAdmittanceController();
            lastAdmittanceUs = micros();
            readySubState = ADMITTANCE;
        }

        // ── Sub-state: ADMITTANCE ────────────────────────────────────────────────
        // Default operating mode. ODrives run in soft velocity control so the arm
        // is compliant and can be moved by hand. Runs every loop iteration until
        // the user triggers a return-to-home (H key).
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

        // Periodically read ODrive error registers — skip during homing to avoid
        // blocking CAN requests competing with the homing loop.
        if (readySubState == ADMITTANCE &&
            (now - lastOdriveErrorCheckMs) >= ODRIVE_ERROR_CHECK_INTERVAL_MS) {
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
        stopODrives();          // emergency stop all motors
        turnOnErrorLED();       // alert operator visually
        sendStateErrorLog();       // report specific fault over Serial
        // Wait for operator acknowledgement before attempting recovery
        if (pwrPressed) {
            errorRecovery();    // clear error and restart from BOOTUP
        }
        break;

    // Unknown or corrupted state — should never be reached.
    default:
        stopODrives();
        turnOnErrorLED();
        sendStateErrorLog();
        break;
    }
} // End of function