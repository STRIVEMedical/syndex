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
static const uint32_t IDLE_ENCODER_POLL_INTERVAL_MS = 50;

// ── READY sub-state — file-scope so variables survive state re-entries ────────
// Using file-scope (not static locals) means resetReadyState() can zero them on
// every entry, preventing stale values if the system exits and re-enters READY.
enum ReadySubState { ADMITTANCE, MOVING_TO_HOME, RESTORING };
static ReadySubState readySubState          = ADMITTANCE;
static bool          readyTelemInit         = false;
static bool          enteredReady           = false;
static uint32_t      lastJointTelemMs       = 0;
static uint32_t      lastStatusTelemMs      = 0;
static uint32_t      lastIdleEncoderPollMs  = 0;
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
static const uint32_t ODRIVE_ERROR_CHECK_INTERVAL_MS = 100000;
static const uint32_t HOMING_LOG_INTERVAL_MS      = 500;

static void resetReadyState() {
    readySubState          = ADMITTANCE;
    readyTelemInit         = false;
    enteredReady           = false;
    lastJointTelemMs       = 0;
    lastStatusTelemMs      = 0;
    lastIdleEncoderPollMs  = 0;
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

/* Terminates immediately, returning FALSE if target ODrive is providing valid connection.
    Otherwise, prints failing node to terminal with invalid connection's info */
static bool reportOdriveHeartbeatFault(const ODriveUserData& data, uint8_t node_id, uint32_t now_ms) {
    if (odriveHeartbeatFresh(data, now_ms)) {
        return false;
    }

    SerialUSB1.print("[ODRIVE] Node ");
    SerialUSB1.print(node_id);
    if (data.received_heartbeat) {
        SerialUSB1.print(" heartbeat stale, age_ms=");
        SerialUSB1.println(static_cast<uint32_t>(now_ms - data.last_heartbeat_ms));
    } else {
        SerialUSB1.println(" heartbeat never received");
    }
    return true;
}

static bool verifyOdriveHeartbeatsOrError() {
    uint32_t now = millis();
    bool fault = false;
    fault |= reportOdriveHeartbeatFault(odrv0_user_data, 0, now);
    fault |= reportOdriveHeartbeatFault(odrv1_user_data, 1, now);
    fault |= reportOdriveHeartbeatFault(odrv2_user_data, 2, now);

    if (fault) {
        setError(ODRIVE_ERROR);
        return false;
    }
    return true;
}

static const char* wireErrorName(uint8_t error) {
    switch (error) {
    case 0: return "success";
    case 1: return "data too long";
    case 2: return "address NACK";
    case 3: return "data NACK";
    case 4: return "other/timeout";
    case 5: return "FIFO error";
    default: return "unknown";
    }
}

static void printWireResult(const char* prefix, uint8_t address, uint8_t error) {
    SerialUSB1.print(prefix);
    SerialUSB1.print(" addr=0x");
    SerialUSB1.print(address, HEX);
    SerialUSB1.print(" wireErr=");
    SerialUSB1.print((int)error);
    SerialUSB1.print(" (");
    SerialUSB1.print(wireErrorName(error));
    SerialUSB1.println(")");
}

static uint8_t probeI2CAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

static bool scanMuxAddressRange() {
    SerialUSB1.println("[I2C][BOOTUP] Scanning possible PCA/TCA9548A addresses 0x70-0x77...");
    bool found = false;
    bool foundExpected = false;
    for (uint8_t address = 0x70; address <= 0x77; address++) {
        uint8_t error = probeI2CAddress(address);
        if (error == 0) {
            found = true;
            if (address == TCA_ADDR) {
                foundExpected = true;
            }
            SerialUSB1.print("[I2C][BOOTUP] Device ACK at possible PCA/TCA address 0x");
            SerialUSB1.println(address, HEX);
        } else {
            printWireResult("[I2C][BOOTUP] No ACK", address, error);
        }
    }
    if (!found) {
        SerialUSB1.println("[I2C][BOOTUP] No PCA/TCA9548A address responded in 0x70-0x77");
        SerialUSB1.println("[I2C][BOOTUP] Check mux power/GND, pullups, SDA=18/SCL=19 on Wire, RESET pin high, and A0-A2 address pins");
    }
    return foundExpected;
}

static uint8_t probeMuxWithClock(uint32_t clock_hz) {
    Wire.setClock(clock_hz);
    delay(5);
    SerialUSB1.print("[I2C][BOOTUP] Probing PCA/TCA9548A at ");
    SerialUSB1.print(clock_hz / 1000);
    SerialUSB1.println(" kHz");
    return probeI2CAddress(TCA_ADDR);
}

static uint8_t selectTcaChannelDiagnostic(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    return Wire.endTransmission();
}

static void scanSelectedMuxChannel(uint8_t jointIndex, uint8_t channel) {
    SerialUSB1.print("[I2C][BOOTUP] Joint ");
    SerialUSB1.print((int)jointIndex);
    SerialUSB1.print(" scanning selected mux ch");
    SerialUSB1.print((int)channel);
    SerialUSB1.println(" for downstream devices...");

    bool found = false;
    for (uint8_t address = 0x08; address <= 0x77; address++) {
        uint8_t error = probeI2CAddress(address);
        if (error != 0) continue;

        found = true;
        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)jointIndex);
        SerialUSB1.print(" mux ch");
        SerialUSB1.print((int)channel);
        SerialUSB1.print(" device ACK addr=0x");
        SerialUSB1.println(address, HEX);
    }

    if (!found) {
        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)jointIndex);
        SerialUSB1.print(" mux ch");
        SerialUSB1.print((int)channel);
        SerialUSB1.println(" no downstream devices ACKed");
    }
}

static void scanAllMuxChannelsForAS5600(uint8_t failingJointIndex, uint8_t failingChannel) {
    SerialUSB1.print("[I2C][BOOTUP] Joint ");
    SerialUSB1.print((int)failingJointIndex);
    SerialUSB1.print(" AS5600 not found on expected mux ch");
    SerialUSB1.print((int)failingChannel);
    SerialUSB1.println("; scanning all mux channels for addr=0x36...");

    bool found = false;
    for (uint8_t channel = 0; channel < 8; channel++) {
        uint8_t muxError = selectTcaChannelDiagnostic(channel);
        if (muxError != 0) {
            SerialUSB1.print("[I2C][BOOTUP] mux ch");
            SerialUSB1.print((int)channel);
            SerialUSB1.print(" select failed ");
            printWireResult("", TCA_ADDR, muxError);
            continue;
        }

        delayMicroseconds(200);
        uint8_t error = probeI2CAddress(AS5600_ADDR);
        if (error == 0) {
            found = true;
            SerialUSB1.print("[I2C][BOOTUP] AS5600 ACK at mux ch");
            SerialUSB1.println((int)channel);
        } else {
            SerialUSB1.print("[I2C][BOOTUP] AS5600 missing at mux ch");
            SerialUSB1.print((int)channel);
            SerialUSB1.print(" ");
            printWireResult("", AS5600_ADDR, error);
        }
    }

    if (!found) {
        SerialUSB1.println("[I2C][BOOTUP] No AS5600 responded on any mux channel");
    }
}

static void scanAllMuxChannelsAllAddresses() {
    SerialUSB1.println("[I2C][BOOTUP] Scanning all mux channels for all I2C addresses...");
    for (uint8_t channel = 0; channel < 8; channel++) {
        uint8_t muxError = selectTcaChannelDiagnostic(channel);
        if (muxError != 0) {
            SerialUSB1.print("[I2C][BOOTUP] mux ch");
            SerialUSB1.print((int)channel);
            SerialUSB1.print(" select failed ");
            printWireResult("", TCA_ADDR, muxError);
            continue;
        }

        delayMicroseconds(200);
        bool found = false;
        for (uint8_t address = 0x08; address <= 0x77; address++) {
            uint8_t error = probeI2CAddress(address);
            if (error != 0) continue;

            found = true;
            SerialUSB1.print("[I2C][BOOTUP] mux ch");
            SerialUSB1.print((int)channel);
            SerialUSB1.print(" device ACK addr=0x");
            SerialUSB1.println(address, HEX);
        }

        if (!found) {
            SerialUSB1.print("[I2C][BOOTUP] mux ch");
            SerialUSB1.print((int)channel);
            SerialUSB1.println(" no devices ACKed");
        }
    }
}

static bool readAS5600RawDiagnostic(uint8_t jointIndex, uint8_t channel, uint16_t& raw) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(ANGLE_HIGH);
    uint8_t txError = Wire.endTransmission(false);
    if (txError != 0) {
        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)jointIndex);
        SerialUSB1.print(" mux ch");
        SerialUSB1.print((int)channel);
        SerialUSB1.print(" AS5600 register select failed");
        SerialUSB1.print(" reg=0x");
        SerialUSB1.print(ANGLE_HIGH, HEX);
        SerialUSB1.print(" ");
        printWireResult("", AS5600_ADDR, txError);
        scanSelectedMuxChannel(jointIndex, channel);
        scanAllMuxChannelsForAS5600(jointIndex, channel);
        raw = 0xFFFF;
        return false;
    }

    uint8_t bytesRead = Wire.requestFrom(AS5600_ADDR, 2);
    if (bytesRead < 2 || Wire.available() < 2) {
        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)jointIndex);
        SerialUSB1.print(" mux ch");
        SerialUSB1.print((int)channel);
        SerialUSB1.print(" AS5600 read incomplete bytes=");
        SerialUSB1.println((int)bytesRead);
        raw = 0xFFFF;
        return false;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    raw = ((msb << 8) | lsb) & 0x0FFF;
    return true;
}

static void printBootEncoderValue(uint8_t jointIndex, uint8_t channel, uint16_t raw) {
    float angleDeg = computeAngle(channel, raw);
    Joint* j = getJoint(jointIndex);
    if (j != nullptr) {
        j->rawValue = raw;
        j->angle = angleDeg;
        j->velocity = 0.0f;
    }

    SerialUSB1.print("[I2C][BOOTUP] Joint ");
    SerialUSB1.print((int)jointIndex);
    SerialUSB1.print(" AS5600 OK mux ch");
    SerialUSB1.print((int)channel);
    SerialUSB1.print(" raw=");
    SerialUSB1.print(raw);
    SerialUSB1.print(" angle_deg=");
    SerialUSB1.println(angleDeg, 2);
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
    SerialUSB1.println("[I2C][BOOTUP] Verifying PCA/TCA9548A mux and external AS5600 encoders");

    uint8_t error = probeMuxWithClock(400000);
    if (error != 0) {
        printWireResult("[I2C][BOOTUP] PCA/TCA9548A ACK failed", TCA_ADDR, error);
        error = probeMuxWithClock(100000);
    }
    if (error != 0) {
        printWireResult("[I2C][BOOTUP] PCA/TCA9548A ACK failed", TCA_ADDR, error);
        if (!scanMuxAddressRange()) {
            setError(I2C_ERROR);
            return false;
        }
        SerialUSB1.println("[I2C][BOOTUP] PCA/TCA9548A recovered during scan; continuing");
        error = 0;
    }
    printWireResult("[I2C][BOOTUP] PCA/TCA9548A ACK OK", TCA_ADDR, error);
    Wire.setClock(400000);
    scanAllMuxChannelsAllAddresses();

    uint8_t expectedExternalEncoders = 0;
    uint8_t detectedExternalEncoders = 0;
    uint8_t missingExternalEncoders = 0;

    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);
        if (j == nullptr) {
            SerialUSB1.print("[I2C][BOOTUP] Null joint pointer at index ");
            SerialUSB1.println((int)i);
            missingExternalEncoders++;
            continue;
        }

        if (j->use_onboard_encoder || j->sensor_channel == INACTIVE_CHANNEL) {
            SerialUSB1.print("[I2C][BOOTUP] Joint ");
            SerialUSB1.print((int)i);
            SerialUSB1.println(" uses ODrive/onboard encoder; skipping I2C encoder check");
            continue;
        }

        expectedExternalEncoders++;

        if (j->sensor_channel > 7) {
            SerialUSB1.print("[I2C][BOOTUP] Invalid mux channel ");
            SerialUSB1.print((int)j->sensor_channel);
            SerialUSB1.print(" (joint index ");
            SerialUSB1.print((int)i);
            SerialUSB1.println(")");
            missingExternalEncoders++;
            continue;
        }

        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)i);
        SerialUSB1.print(" selecting mux channel ");
        SerialUSB1.println((int)j->sensor_channel);

        uint8_t muxError = selectTcaChannelDiagnostic(j->sensor_channel);
        if (muxError != 0) {
            SerialUSB1.print("[I2C][BOOTUP] Joint ");
            SerialUSB1.print((int)i);
            SerialUSB1.print(" mux channel ");
            SerialUSB1.print((int)j->sensor_channel);
            SerialUSB1.print(" select failed ");
            printWireResult("", TCA_ADDR, muxError);
            j->rawValue = 0xFFFF;
            missingExternalEncoders++;
            continue;
        }
        SerialUSB1.print("[I2C][BOOTUP] Joint ");
        SerialUSB1.print((int)i);
        SerialUSB1.print(" mux channel ");
        SerialUSB1.print((int)j->sensor_channel);
        SerialUSB1.println(" select OK");

        delayMicroseconds(200);
        uint16_t raw = 0xFFFF;
        if (!readAS5600RawDiagnostic(i, j->sensor_channel, raw)) {
            j->rawValue = 0xFFFF;
            missingExternalEncoders++;
            continue;
        }

        printBootEncoderValue(i, j->sensor_channel, raw);

        detectedExternalEncoders++;
    }

    if (detectedExternalEncoders != expectedExternalEncoders) {
        SerialUSB1.print("[I2C][BOOTUP] Encoder count mismatch expected=");
        SerialUSB1.print((int)expectedExternalEncoders);
        SerialUSB1.print(" detected=");
        SerialUSB1.println((int)detectedExternalEncoders);
        SerialUSB1.print("[I2C][BOOTUP] Continuing with missing external encoder count=");
        SerialUSB1.println((int)missingExternalEncoders);
        return true;
    }

    SerialUSB1.print("[I2C][BOOTUP] External encoder check passed count=");
    SerialUSB1.println((int)detectedExternalEncoders);
    return true;
}

/*
 * Verifies all LED pins are initialized and responding correctly.
 * Calls Led::setup() to configure all pins as OUTPUT, then writes HIGH
 * to each LED and reads back to confirm the pin responded.
 * Returns true if all three LEDs respond correctly, false if any fail.
 */
bool verifyLED(){
    // Test power LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::powerLED);
    int powerVal = digitalRead(Led::powerLED.pin);
    SerialUSB1.print("[DEBUG] Power LED pin read: ");
    SerialUSB1.println(powerVal);
    if (powerVal != HIGH) {
        SerialUSB1.println("[DEBUG] Power LED verification failed");
        setError(LED_ERROR);
        return false;
    }
    // Test data LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::dataLED);
    int dataVal = digitalRead(Led::dataLED.pin);
    SerialUSB1.print("[DEBUG] Data LED pin read: ");
    SerialUSB1.println(dataVal);
    if (dataVal != HIGH) {
        SerialUSB1.println("[DEBUG] Data LED verification failed");
        setError(LED_ERROR);
        return false;
    }
    // Test error LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::errorLED);
    int errorVal = digitalRead(Led::errorLED.pin);
    SerialUSB1.print("[DEBUG] Error LED pin read: ");
    SerialUSB1.println(errorVal);
    if (errorVal != HIGH) {
        SerialUSB1.println("[DEBUG] Error LED verification failed");
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
    return verifyOdriveHeartbeatsOrError();
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
 * External encoder read failures leave the last known angle in place and are
 * reported over SerialUSB1 by readJointAngles(); they do not latch ERROR_STATE.
 * Called every loop cycle in READY state.
 */
void enableI2CPacketSend()
{
    telemJointDataPayload data{};

    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);

        if (j == nullptr) {
            sendErrorMessage("I2C error: invalid joint pointer");
            continue;
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

    if (!verifyOdriveHeartbeatsOrError()) {
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
    SerialUSB1.println("[DEBUG] turnOnErrorLED() called");
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
    turnOnErrorLED();
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
        // bool pwrPressed = powerButtonWasPressed();
        // if (pwrPressed) {
        //     SerialUSB1.println("[DEBUG] Power button pressed");
        // }
        // if (pwrPressed && (currState != BOOTUP && currState != ERROR_STATE && currState != POWERINGOFF)) {
        //     currState = POWERINGOFF;
        //     return;
        // }
        // if (currState == IDLE || currState == HOMING || currState == READY) {
        //     if (!verifyOdriveHeartbeatsOrError()) {
        //         currState = ERROR_STATE;
        //         return;
        //     }
        // }

  switch (currState) {
    // Verify all hardware before allowing any operation.
    // Success: power LED on, transition to IDLE.
    // Failure: transition to ERROR_STATE.
    case BOOTUP:
        // if (!pwrPressed) {
        //     break;
        // }
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
        // static uint32_t lastIdleErrorCheckMs = 0;
        // if ((millis() - lastIdleErrorCheckMs) >= 5000) {
        //     pumpEvents(can_intf);
        //     printOdriveError(&odrv0, 0);
        //     printOdriveError(&odrv1, 1);
        //     printOdriveError(&odrv2, 2);
        //     lastIdleErrorCheckMs = millis();
        // }
        // Simple blink test for pin 13 (error LED/onboard LED)
        // static uint32_t lastBlinkMs = 0;
        // static bool ledState = false;
        // uint32_t nowBlink = millis();
        // if (nowBlink - lastBlinkMs > 500) { // 500ms interval
        //     lastBlinkMs = nowBlink;
        //     ledState = !ledState;
        //     pinMode(13, OUTPUT);
        //     digitalWrite(13, ledState ? HIGH : LOW);
        // }
        uint32_t now = millis();
        if ((now - lastIdleEncoderPollMs) >= IDLE_ENCODER_POLL_INTERVAL_MS) {
            readJointAngles();
            lastIdleEncoderPollMs = now;
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
                bool armed = odriveHeartbeatFresh(*joints[i].user_data, millis()) &&
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
                    bool reArmed = odriveHeartbeatFresh(*joints[i].user_data, millis()) &&
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
            if (currState == ERROR_STATE) {
                break;
            }
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
        if (millis() <= 5500) {
        stopODrives();          // emergency stop all motors
        turnOnErrorLED();       // alert operator visually
        sendStateErrorLog();       // report specific fault over Serial
        }
        // Wait for operator acknowledgement before attempting recovery
        // if (pwrPressed) {
        //     errorRecovery();    // clear error and restart from BOOTUP
        // }
        break;

    // Unknown or corrupted state — should never be reached.
    default:
        if (millis()<= 5500) {
        stopODrives();
        turnOnErrorLED();
        sendStateErrorLog();
        }
        break;
    }
} // End of function
