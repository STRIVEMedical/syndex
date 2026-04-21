#include "states.h"
#include "odrive.h"
#include "LEDs.h"
#include "comms.h"
#include "buttons.h"
#include "USB.h"
#include "Wire.h"
#include "joint.h"
#include "admittance_controller.h"
#include "errors.h"

state_e currState = BOOTUP;
errorCode_e currError = NO_ERROR;

// Telemetry pacing to avoid saturating the USB receive queue on host.
static const uint32_t JOINT_TELEM_INTERVAL_MS = 5; 
static const uint32_t STATUS_TELEM_INTERVAL_MS = 100; // 10 Hz

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
#ifdef ODRIVE_FULL
    odrivesHealthy &= printOdriveError(&odrv0, 0);
#endif
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
    // Initialize all LED pins as OUTPUT
    Led::setup();
    // Test power LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::powerLed);
    if (digitalRead(Led::powerLed.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    // Test data LED — write HIGH and confirm pin responds
    ONToggleLED(&Led::dataLed);
    if (digitalRead(Led::dataLed.pin) != HIGH) {
        setError(LED_ERROR);
        return false;
    }
    // Test error LED — write HIGH and confirm pin responds
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
    return pingReceived;  // Check if ping was received from host
}

//Checks whether the ODrive comms are online
bool verifyODriveComms(){
    // Check all ODrives are still sending heartbeats over CAN
    if (
#ifdef ODRIVE_FULL
        !odrv0_user_data.received_heartbeat ||
#endif
        !odrv1_user_data.received_heartbeat ||
        !odrv2_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        return false;       // An ODrive stopped responding
    }
    else {
        return true;
    }
}

/*
 * Convenience wrapper that checks all three connection layers are healthy.
 * Checks in order: USB → ODrive CAN → I2C encoders
 * Sets specific error code for whichever layer fails first.
 * Returns true only if all three layers pass, false if any fail.
 */
bool allConnectionsReady()
{
    // Check USB connection to host PC
    if (!verifyUSB()) {
        setError(CONNECTION_ERROR);
        return false;
    }
    // Check ODrive CAN heartbeats
    if (!verifyODriveComms()) {
        setError(ODRIVE_ERROR);
        return false;
    }
    // Checks all 7 I2C encoders
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
// payload struct to hold angle and velocity data for all joints
    telemJointDataPayload data;

    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);

        // check if joint ID is valid
        if (j == nullptr) {
            setError(I2C_ERROR);
            sendErrorMessage("I2C error: invalid joint pointer");

            currState = ERROR_STATE;
            return; // exit immediately — invalid joint
        }

        // External encoder joints use raw AS5600 values; 0xFFFF indicates read failure.
        if (!j->use_onboard_encoder && j->rawValue == 0xFFFF) {
            setError(I2C_ERROR);
            // notify host PC that an encoder failure occurred
            sendErrorMessage("I2C error: external encoder read failed");
            // safe the system and exit function immediately
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
 * Called every loop cycle in READY state.
 * TODO: fill status payload fields once confirmed with firmware team.
 */
void enableODrivePacketSend()
{
    // payload struct to hold ODrive status data
    telemStatusPayload status{};

    // check ODrive heartbeats are still active over CAN
    if (
#ifdef ODRIVE_FULL
        !odrv0_user_data.received_heartbeat ||
#endif
        !odrv1_user_data.received_heartbeat ||
        !odrv2_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        // notify host PC that ODrive communication failed
        packet errPacket(ERROR_MESSAGE);
        std::vector<uint8_t> errBuffer = errPacket.serialize();
        Serial.write(errBuffer.data(), errBuffer.size());
        // safe the system and exit function immediately
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
 * Graceful shutdown prevents motors from dropping torque suddenly,
 * which could cause the arm to fall or jerk unexpectedly.
 */

void stopODrives(){
    emergencyStop();
}

/*
 * Turns off all LED indicators during shutdown sequence.
 * Called after ODrives are powered off as part of orderly shutdown.
 */
void powerOffPeripherals(){
    OFFToggleLED(&Led::powerLed);
    OFFToggleLED(&Led::dataLed);
    OFFToggleLED(&Led::errorLed);
}


/*
* Turns on the error LED to visually alert the operator of a fault.
* Error LED remains on until errorRecovery() is called.
*/
void turnOnErrorLED(){
    ONToggleLED(&Led::errorLed);
}

/*
 * Resets system state after operator acknowledges and resolves the fault.
 * Clears the error flag, turns off error LED, and returns to BOOTUP
 * to rerun all hardware verification before resuming operation.
 * Only called after operator presses powerButton to acknowledge fault.
 */
void errorRecovery(){
    clearError();                  // reset currError to NO_ERROR
    OFFToggleLED(&Led::errorLed);  // turn off error LED
    currState = BOOTUP;            // restart verification from beginning
}

/*
 * Stores the current error code when a fault is detected.
 * Called immediately before transitioning to ERROR_STATE so
 * sendStateErrorLog() can report the specific cause of the fault.
 */
void setError(errorCode_e err) {
    currError = err;
}

/*
 * Returns the current error code.
 * Useful for external files or PC requests to query what went wrong.
 */
errorCode_e getError() {
    return currError;
}

/*
 * Resets the error code back to NO_ERROR.
 * Called inside errorRecovery() after operator acknowledges the fault.
 */
void clearError(){
    currError = NO_ERROR;
}

/*
 * Returns true if any error is currently active, false if system is healthy.
 * Checks whether currError is anything other than NO_ERROR.
 */
bool errorCheck(){
    return currError != NO_ERROR;
}





/*
 * Transmits a specific error message over Serial based on the current error code.
 * Called in ERROR_STATE to report the fault to the operator or host PC.
 * Each error type has a distinct message for easy fault diagnosis.
 */
void sendStateErrorLog() {
    switch (currError)
    {
    // ODrive hardware or CAN communication failure
    case ODRIVE_ERROR:
        SerialUSB1.println("ERROR: ODRIVE FAILURE");
        break;
    
    // I2C bus or encoder communication failure
    case I2C_ERROR:
        SerialUSB1.println("ERROR: I2C FAILURE");
        break;

    // LED pin not responding during bootup verification
    case LED_ERROR:
        SerialUSB1.println("ERROR: LED FAILURE");
        break;

    // USB or host PC connection failure
    case CONNECTION_ERROR:
        SerialUSB1.println("ERROR: CONNECTION FAILURE");
        break;

    // No error currently active — nothing to report
    case NO_ERROR:
        break;

    // Unknown or unhandled error type
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
 *   ERROR_STATE --> ERROR_STATE  (latched fault until device reset)
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
            break;  // stop here, don't continue to homing check
        }
        else if (isHomed()) {
            currState = READY;  // skip homing, arm position is known
        }
        else {
            currState = HOMING; // need to establish arm home position
        }
        break;

    // Drive arm to home position using position control.
    // startHoming() sends command once via static flag.
    // verifyHoming() polls until complete, then transition to READY.
    case HOMING:
    {
        static bool homingStarted = false;
        if (!homingStarted) {
            startHoming();       // send position commands once
            homingStarted = true;
        }
        if (verifyHoming()) {    // poll each loop until joints settle
            homingStarted = false;  // reset for next time (e.g. after error recovery)
            currState = READY;
        }
        break;
    }
    // Normal operating state — admittance control always running.
    // data LED turns on once on entry via static flag.
    case READY:
    {
        // FOR SENSING
        static bool readyTelemInit = false;
        static bool enteredReady = false;
        static uint32_t lastJointTelemMs = 0;
        static uint32_t lastStatusTelemMs = 0;
        static uint32_t lastAdmittanceUs = 0;
        static uint32_t lastOdriveErrorCheckMs = 0;
        static const uint32_t ODRIVE_ERROR_CHECK_INTERVAL_MS = 500;

        if (!enteredReady) {
            ONToggleLED(&Led::dataLed); // data LED on = system operational
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

        // Sense first, then run admittance update in the same READY cycle.
        readJointAngles();
        uint32_t nowUs = micros();
        float dt = (nowUs - lastAdmittanceUs) * 1e-6f;
        lastAdmittanceUs = nowUs;
        stepAdmittanceController(dt);

        uint32_t now = millis();
        if ((now - lastJointTelemMs) >= JOINT_TELEM_INTERVAL_MS) {
            enableI2CPacketSend();
            lastJointTelemMs = now;
        }

        if ((now - lastStatusTelemMs) >= STATUS_TELEM_INTERVAL_MS) {
            enableODrivePacketSend();
            lastStatusTelemMs = now;
        }

        // Periodically read ODrive error registers so faults show up in serial
        // (the red flashing LED won't tell you which fault it is).
        if ((now - lastOdriveErrorCheckMs) >= ODRIVE_ERROR_CHECK_INTERVAL_MS) {
#ifndef ODRIVE_FULL
            printOdriveError(&odrv1, 1);
            printOdriveError(&odrv2, 2);
#else
            printOdriveError(&odrv0, 0);
            printOdriveError(&odrv1, 1);
            printOdriveError(&odrv2, 2);
#endif
            lastOdriveErrorCheckMs = now;
        }
        break;
    }

    // Controlled shutdown sequence.
    // Motors idled gracefully before power is cut.
    case POWERINGOFF:
        stopODrives();      // gracefully idle all ODrive motors
        powerOffPeripherals();  // turn off all LEDs
        break;

    // Safe all hardware immediately and report fault.
    // Fault is latched to avoid auto-restart loops and noisy serial output.
    case ERROR_STATE:
    {
        static bool errorLatched = false;
        if (!errorLatched) {
            stopODrives();          // emergency stop all motors once on entry
            turnOnErrorLED();       // alert operator visually
            sendStateErrorLog();    // report specific fault over Serial once
            errorLatched = true;
        }
        break;
    }

    // Unknown or corrupted state — should never be reached.
    // Immediately safe all hardware and transition to ERROR_STATE.
    default:
        stopODrives();
        turnOnErrorLED();
        sendStateErrorLog();
        break;
    }
}