#include "comms.h"
#include "odrive.h"
#include "ODriveFlexCAN.hpp"


/**
 * @brief Initializes all communication subsystems for the surgical arm
 * 
 * @return true if all subsystems initialize successfully, false otherwise
 * 
 * @usage Called once at system startup before any ODrive initialization
 * @note Initializes in this order:
 *   1. Serial debug interface (115200 baud)
 *   2. CAN bus communication
 *   3. I2C multiplexer and AS5600 encoders
 * @warning System will halt if initialization fails
 */
bool initCommunications() {
  //Set baude rate for serial monitor
  Serial.begin(115200);
  delay(200);

  //init CAN
  if (!setupCan()) {
    Serial.println("CAN init failed");
    return false;
  }

  // Initialize I2C sensors (AS5600 absolute encoders via TCA9548A mux)
  setupI2C();

  Serial.println("CAN & I2C READY");
  return true;
}


/*--------------------------------i2c-----------------------------*/


// Define globals declared in i2c.h
float zeroOffset[NUM_ENCODERS] = {0, 0, 0, 0};   // per-sensor zeroing
long turns[NUM_ENCODERS] = {0, 0, 0, 0};         // multi-turn tracking
int lastRaw[NUM_ENCODERS] = {0, 0, 0, 0};

/*
Selects channel `ch` on the TCA9548A I2C multiplexer by writing a bitmask 
to its control register (only one channel active at a time)
*/
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/*
Reads the 12-bit raw angle value from the AS5600 encoder by requesting 
two bytes (high and low) from angle registers 0x0E and 0x0F
*/
uint16_t readRawAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_HIGH);                 // ANGLE register MSB
  Wire.endTransmission(false);      // repeated start

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  return ((msb << 8) | lsb) & 0x0FFF; // 12-bit angle
}

/*
Converts a raw AS5600 value (0–4095) to an absolute angle in degrees,
accounting for full rotation wraps (multi-turn) and applying a zero offset
*/
float computeAngle(int sensorID, uint16_t raw) {

  // Detect forward wrap
  if (raw < 100 && lastRaw[sensorID] > 4000)
    turns[sensorID] += 1;

  // Detect backward wrap
  if (raw > 4000 && lastRaw[sensorID] < 100)
    turns[sensorID] -= 1;

  lastRaw[sensorID] = raw;

  float angle = (raw * 360.0f / 4096.0f) + (turns[sensorID] * 360.0f);

  angle -= zeroOffset[sensorID];
  return angle;
}

/*
Initializes I2C communication and sets up the serial interface
for debugging or streaming encoder data to a host
*/
void setupI2C() {
  Wire.begin();          // SDA=18, SCL=19
  Wire.setClock(400000);
}


/*--------------------------------i2c-----------------------------*/


/*--------------------------------CAN-----------------------------*/
// CAN interface for Teensy 4.1 (CAN1 port)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// ODrive instances with unique node IDs
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);  // Node ID 0
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);  // Node ID 1
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);  // Node ID 2

// Array of ODrive pointers for easy iteration
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2};


// User data structures for storing ODrive status information
ODriveUserData odrv0_user_data;  // Status data for ODrive 0
ODriveUserData odrv1_user_data;  // Status data for ODrive 1
ODriveUserData odrv2_user_data;  // Status data for ODrive 2

/**
 * @brief Initializes the CAN bus interface with ODrive-compatible settings
 * 
 * @return true if CAN initialization succeeds, false otherwise
 * 
 * @usage Call once during system initialization
 * @note Configures:
 *   - 250kbps baud rate (ODrive default)
 *   - FIFO buffer for efficient message handling
 *   - Interrupt-driven reception
 *   - Callback routing for all incoming messages
 */
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

/**
 * @brief Callback function for ODrive heartbeat messages
 * 
 * @param msg Heartbeat message containing axis state and error codes
 * @param user_data Pointer to ODriveUserData structure for this ODrive
 * 
 * @usage Automatically called by ODriveCAN library on heartbeat reception
 * @note Heartbeat messages are sent periodically by ODrive (typically 10-100Hz)
 * @important Updates:
 *   - Last known axis state (idle, closed loop, etc.)
 *   - Connection status flag
 *   - Error conditions if present
 */
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_heartbeat = msg;
  ud->received_heartbeat = true;
}

/**
 * @brief Callback function for ODrive encoder feedback messages
 * 
 * @param msg Encoder estimates message containing position and velocity
 * @param user_data Pointer to ODriveUserData structure for this ODrive
 * 
 * @usage Automatically called by ODriveCAN library on feedback reception
 * @note Feedback rate should match control loop frequency (typically 1-10kHz)
 * @important Updates:
 *   - Motor encoder position (in turns)
 *   - Motor encoder velocity (in turns/s)
 *   - Used for closed-loop control monitoring
 */
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* ud = (ODriveUserData*)user_data;
  ud->last_feedback = msg;
  ud->received_feedback = true;
}

/**
 * @brief Routes incoming CAN messages to all registered ODrive instances
 * 
 * @param msg Raw CAN message received by FlexCAN_T4
 * 
 * @usage Registered as callback to FlexCAN_T4's onReceive
 * @note Each ODrive instance filters messages based on its node ID
 * @important Called for EVERY CAN message - keep efficient
 */
void onCanMessage(const CAN_message_t& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}