#include "states.h"
#include "odrive.h"
#include "LEDs.h"
#include "comms.h"
#include "buttons.h"
#include "USB.h"
#include "Wire.h"
#include "joint.h"
#include "errors.h"

state_e currState = BOOTUP;
errorCode_e currError = NO_ERROR;

/*
 * Verifies ODrive system is initialized and all ODrives are online.
 * Step 1: Initialize CAN and I2C communication layer via initOdriveSystem()
 * Step 2: Initialize all ODrive instances and wait for heartbeats via initMultiOdrives()
 * Returns true if all ODrives are online, false if any layer fails.
 */
bool verifyODrive(){
     if (!initCommunications()) {
        setError(CONNECTION_ERROR); // CAN or I2C layer failed
        return false;
    }
    if (!initMultiOdrives()) {
        setError(ODRIVE_ERROR); // CAN fine but ODrive not responding
        return false;
    }
    return true;
}
/*
 * Verifies I2C bus is initialized by checking if the TCA9548A multiplexer
 * responds on the bus. The mux is the gateway to all 7 AS5600 encoders —
 * if the mux is unreachable, no encoder data can be read.
 * Returns true if mux acknowledges, false if bus is down or mux not found.
 */
bool verifyI2C(){
    Wire.beginTransmission(TCA_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        setError(I2C_ERROR);
        return false;
    }
    else {
        return true;
    }
}
/*
 * Verifies all buttons are in their default unpressed state during bootup.
 * Buttons are active-LOW (INPUT_PULLUP) — LOW means pressed, HIGH means unpressed.
 * A button held down during startup is unexpected and could indicate:
 *   - Accidental press
 *   - Stuck button
 *   - Wiring short
 * Returns true if all buttons are unpressed, false if any button is held down.
 */
bool verifyButton(){
    // Ensure GPIO state used by boot checks is initialized.
    Buttons::setup();

    if (digitalRead(buttonPins::powerButton.pin) == LOW ||
        digitalRead(buttonPins::autoHoming.pin) == LOW ||
        digitalRead(buttonPins::triggerButton.pin) == LOW ||
        digitalRead(buttonPins::toolSelect.pin) == LOW) {
        setError(BUTTON_ERROR);
        return false;
     }
     else {
        return true;
     }
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
* Polls serial buffer for an incoming CMD_PING packet from the host PC.
* The host sends CMD_PING (0x01) to initiate connection with the device.
* Returns true if a valid PING packet is detected, false otherwise.
* TODO: Implement using USB.cpp packet parsing once USB.cpp is available.
*/
bool pollCmdPing(){
    if (Serial.available() <= 0) {
        return false;
    }

    // Minimal ping check: host sends raw CMD_PING byte.
    const int incoming = Serial.read();
    return incoming == CMD_PING;
}

/*
* Sends a RESP_PONG (0x81) packet back to the host PC in response to CMD_PING.
* Confirms to the host that the device is alive and ready to connect.
* TODO: Implement using USB.cpp packet serialization once USB.cpp is available.
*/
void sendCmdPong(){
    packet pong(RESP_PONG);
    std::vector<uint8_t> bytes = pong.serialize();
    Serial.write(bytes.data(), bytes.size());
}

/*
* Verifies USB serial connection is open and host is actively connected.
* Distinct from BOOTUP hardware checks — this confirms the communication
* link to the host PC is live at the moment of connection.
* TODO: Implement once USB.cpp is available.
*/
bool verifyUSB(){
    if (!Serial) {
        setError(CONNECTION_ERROR);
        return false;   // Host is not connected
    }
    else {
        return true;    // Host is connected
    }
}

//Checks whether the ODrive comms are online
bool verifyODriveComms(){
    // Check all ODrives are still sending heartbeats over CAN
    if (!odrv0_user_data.received_heartbeat ||
        !odrv1_user_data.received_heartbeat ||
        !odrv2_user_data.received_heartbeat) {
        setError(ODRIVE_ERROR);
        return false;       // An ODrive stopped responding
    }
    else {
        return true;
    }
}

bool verifyConnectedI2CDevices() {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);
        if (j == nullptr) {
            setError(I2C_ERROR);
            return false;
        }

        if (j->use_onboard_encoder || j->sensor_channel == INACTIVE_CHANNEL) {
            continue;
        }

        tcaSelect(j->sensor_channel);
        uint16_t raw = readRawAS5600();
        if (raw == 0xFFFF) {
            setError(I2C_ERROR);
            return false;
        }
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
    if (!verifyConnectedI2CDevices()) {
        setError(I2C_ERROR);
        return false;
    }
    return true;
}

//Checks whether simulation is already homed
bool isHomed(){
    //Placeholder for homing logic
    return false;
}

//Initiates the homing proccess
void startHoming(){
//Placeholder for Homing logic

}

//Checks whether the homing proccess has been homed
bool verifyHoming(){
//Placeholder for Homing logic
    return false;
}

/*
 * Reads all joint encoder channels via TCA9548A multiplexer and
 * streams joint angle data to host PC as a TELEM_JOINT_DATA packet.
 * If any encoder fails during reading, sends ERROR_MESSAGE to host
 * and transitions to ERROR_STATE immediately.
 * Called every loop cycle in READY state.
 */
void enableI2CPacketSend()
{
// payload struct to hold angle and velocity data for all joints
    telemJointDataPayload data;

    // read all joint angles and velocities into Joint structs
    readJointAngles();

    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint* j = getJoint(i);

        // check if joint ID is valid
        if (j == nullptr) {
            setError(I2C_ERROR);
            packet errPacket(ERROR_MESSAGE);
            std::vector<uint8_t> bytes = errPacket.serialize();
            Serial.write(bytes.data(), bytes.size());
            currState = ERROR_STATE;
            return; // exit immediately — invalid joint
        }

        // External encoder joints use raw AS5600 values; 0xFFFF indicates read failure.
        if (!j->use_onboard_encoder && j->rawValue == 0xFFFF) {
            setError(I2C_ERROR);
            // notify host PC that an encoder failure occurred
            packet errPacket(ERROR_MESSAGE);
            std::vector<uint8_t> errBuffer = errPacket.serialize();
            Serial.write(errBuffer.data(), errBuffer.size());
            // safe the system and exit function immediately
            currState = ERROR_STATE;
            return;
        }

        buildTelemJointPayload(data, i, j->angle, j->velocity);
    }

    // all joints read successfully — build and send telemetry packet to host PC
    packet I2CPacket(TELEM_JOINT_DATA, data);
    // serialize converts packet object into raw byte stream for USB transmission
    std::vector<uint8_t> I2CBuffer = I2CPacket.serialize();
    // transmit byte stream to host PC over USB serial
    Serial.write(I2CBuffer.data(), I2CBuffer.size());
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
    if (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat || !odrv2_user_data.received_heartbeat) {
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

    // all ODrives healthy - build and sent status telemetry 
    packet odrvPacket(TELEM_STATUS, status);
    // serialize converts packet object into raw byte stream for USB transmission
    std::vector<uint8_t> statusBuffer = odrvPacket.serialize();
    // transmit byte stream to PC over USB serial
    Serial.write(statusBuffer.data(), statusBuffer.size());
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
 * Cuts system power after ODrives have been stopped and hardware is safe.
 * Always called after stopODrives()
*/
void endPower(){

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
 * sendErrMessage() can report the specific cause of the fault.
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
void sendErrMessage() {
    switch (currError)
    {
    // ODrive hardware or CAN communication failure
    case ODRIVE_ERROR:
        Serial.println("ERROR: ODRIVE FAILURE");
        break;
    
    // I2C bus or encoder communication failure
    case I2C_ERROR:
        Serial.println("ERROR: I2C FAILURE");
        break;

    // Button stuck or pressed during bootup
    case BUTTON_ERROR:
        Serial.println("ERROR: BUTTON FAILURE");
        break;

    // LED pin not responding during bootup verification
    case LED_ERROR:
        Serial.println("ERROR: LED FAILURE");
        break;

    // USB or host PC connection failure
    case CONNECTION_ERROR:
        Serial.println("ERROR: CONNECTION FAILURE");
        break;

    // No error currently active — nothing to report
    case NO_ERROR:
        break;

    // Unknown or unhandled error type
    default:
    Serial.println("ERROR: UNKNOWN FAILURE");
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
 *   ERROR_STATE --> BOOTUP      (operator acknowledges and presses powerButton)
*/
void stateUpdate()
{
  switch (currState) {
    // Verify all hardware before allowing any operation.
    // Success: power LED on, transition to IDLE.
    // Failure: transition to ERROR_STATE.
    case BOOTUP:
        if (verifyODrive() && verifyI2C() && verifyButton() && verifyLED()) {
            ONToggleLED(&Led::powerLed);   // power LED on = system alive
            OFFToggleLED(&Led::errorLed);  // ensure error LED is off
            currState = IDLE;
        }
        else {
            currState = ERROR_STATE;
        }
        break;

    // Wait for CMD_PING from host PC.
    // PING received: send PONG, transition to CONNECTED.
    case IDLE:
        if (pollCmdPing()) {
            sendCmdPong();      // confirm device is alive to host
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
        startHoming();          // send homing command once
        if (verifyHoming()) {   // poll until homing complete
            currState = READY;
        }
        break;

    // Normal operating state — admittance control always running.
    // data LED turns on once on entry via static flag.
    case READY:
    {
        static bool enteredReady = false;
        if (!enteredReady) {
        ONToggleLED(&Led::dataLed); // data LED on = system operational
        enteredReady = true;
        }
        enableI2CPacketSend();
        enableODrivePacketSend();
        break;
    }

    // Controlled shutdown sequence.
    // Motors idled gracefully before power is cut.
    case POWERINGOFF:
        stopODrives();      // gracefully idle all ODrive motors
        powerOffPeripherals();  // turn off all LEDs
        endPower();             // cut system power
        break;

    // Safe all hardware immediately and report fault.
    // Waits for operator to press powerButton before recovery.
    case ERROR_STATE:
        stopODrives();          // emergency stop all motors
        endPower();             // cut power
        turnOnErrorLED();       // alert operator visually
        sendErrMessage();       // report specific fault over Serial
        // Wait for operator acknowledgement before attempting recovery
        if (digitalRead(buttonPins::powerButton.pin) == LOW) {
                errorRecovery();    // clear error and restart from BOOTUP
        }
        break;

    // Unknown or corrupted state — should never be reached.
    // Immediately safe all hardware and transition to ERROR_STATE.
    default:
        stopODrives();
        endPower();
        turnOnErrorLED();
        sendErrMessage();
        break;
    }
}