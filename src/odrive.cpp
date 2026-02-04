// odrive.cpp
// ODrive CAN communication and control functions for surgical robot arm
// 
// Key Concepts:
// - ODrive operates in different control modes (position/velocity/torque)
// - This implementation focuses on torque control for gravity compensation
// - AS5600 absolute encoders provide position feedback via I2C
// - CAN bus communicates with ODrive motor controllers

#include <Arduino.h>
#include "ODriveCAN.h"
#include "odrive.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include "i2c.h"

// Define global instances declared as extern in odrive.h
// CAN interface for Teensy 4.1 (CAN1 port)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// ODrive instances with unique node IDs
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);  // Node ID 0
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);  // Node ID 1

// Array of ODrive pointers for easy iteration
ODriveCAN* odrives[] = { &odrv0, &odrv1 };

// User data structures for storing ODrive status information
ODriveUserData odrv0_user_data;  // Status data for ODrive 0
ODriveUserData odrv1_user_data;  // Status data for ODrive 1

/* =========================
 * CAN SETUP
 * ========================= */

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

/* =========================
 * ODRIVE STATE AND CONTROL MODE
 * ========================= */

/**
 * @brief Transitions an ODrive to closed-loop position control mode
 * 
 * @param odrv Reference to the ODrive instance to control
 * @param data Reference to the ODrive's status data structure
 * 
 * @usage Used for precise position control during homing sequence
 * @warning This sets position control - not suitable for gravity compensation
 * @note Repeatedly sends state command until ODrive confirms mode change
 */
void enable_closed_loop(ODriveCAN &odrv, ODriveUserData &data) {
  while (data.last_heartbeat.Axis_State !=
         ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    delay(1);
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    for (int i = 0; i < 15; i++) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
}

/**
 * @brief Configures an ODrive for torque control with direct passthrough input
 * 
 * @param odrv Reference to the ODrive instance to configure
 * @param data Reference to the ODrive's status data structure
 * 
 * @usage Required for gravity compensation where motors apply constant torque
 * @note This mode allows:
 *   - Direct torque commands via CAN
 *   - Natural compliance (arm can be moved manually)
 *   - Gravity counter-torque application
 * @warning ODrive must be pre-configured via USB for torque control compatibility
 */
void enable_torque_control(ODriveCAN &odrv, ODriveUserData &data) {
  // Wait until ODrive confirms torque control and passthrough input mode
  while ((data.last_controller_mode.Control_Mode != 
        ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL) && 
        (data.last_input_mode.Input_Mode != 
        ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
    
    odrv.clearErrors();
    delay(1);
    
    // Set control mode to torque control and Set input mode to passthrough (direct torque commands)
    odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    


    // Process CAN messages to update status
    for (int i = 0; i < 15; i++) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  Serial.println("Torque control enabled");
}

/* =========================
 * CAN CALLBACKS
 * ========================= */

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

/* =========================
 * GLOBAL / SYSTEM INIT
 * ========================= */

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
bool initOdriveSystem() {
  Serial.begin(115200);
  delay(200);

  if (!setupCan()) {
    Serial.println("CAN init failed");
    return false;
  }

  // Initialize I2C sensors (AS5600 absolute encoders via TCA9548A mux)
  setupI2C();

  Serial.println("CAN & I2C READY");
  return true;
}

/* =========================
 * PER-ODRIVE INIT (GENERAL)
 * ========================= */

/**
 * @brief Initializes a single ODrive controller with proper configuration
 * 
 * @param odrv Reference to the ODrive instance to initialize
 * @param data Reference to the ODrive's status data structure
 * 
 * @return true if ODrive initializes successfully, false otherwise
 * 
 * @usage Called for each ODrive in the system (typically 7 for 7DOF arm)
 * @note Initialization sequence:
 *   1. Register callbacks for heartbeat and feedback
 *   2. Wait for heartbeat to confirm ODrive presence on CAN bus
 *   3. Enable torque control mode for gravity compensation
 * @warning Blocks until heartbeat received - ensure ODrive is powered
 */
bool initOdrive(ODriveCAN &odrv, ODriveUserData &data) {
  // Register callbacks for status updates
  odrv.onStatus(onHeartbeat, &data);
  odrv.onFeedback(onFeedback, &data);

  // Wait for heartbeat to confirm ODrive is online
  Serial.println("Waiting for ODrive...");
  while (!data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("ODrive Found!");

  // Enable torque control mode (required for gravity compensation)
  Serial.println("Enabling torque control and input passthrough...");
  enable_torque_control(odrv, data);
  Serial.println("ODrive Running!");
  return true;
}

/* =========================
 * INIT MULTIPLE ODRIVES
 * ========================= */

/**
 * @brief Initializes all ODrive controllers in the surgical arm system
 * 
 * @return true if all ODrives initialize successfully, false otherwise
 * 
 * @usage Main initialization function called from setup() in main.cpp
 * @note Current implementation initializes 2 ODrives (for base joints)
 *       Should be expanded to 7 ODrives for full 7DOF control
 * @warning If any ODrive fails to initialize, entire system startup fails
 */
bool initMultiOdrives() {
  // Initialize communication subsystems
  if (!initOdriveSystem()) {
    return false;
  }

  // Initialize ODrive 0 (typically base joint 1 - pulley1)
  if (!initOdrive(odrv0, odrv0_user_data)) {
    return false;
  }
  
  // Initialize ODrive 1 (typically base joint 2 - pulley2)
  if (!initOdrive(odrv1, odrv1_user_data)) {
    return false;
  }

  Serial.println("All ODrives Running!");
  return true;
}

/**
 * @brief Emergency stop function for all ODrives
 * 
 * @usage Called on emergency stop button press or fault detection
 * @note Immediately disables all motors and sets system to safe state
 */
void emergencyStop() {
  for (auto odrive : odrives) {
    odrive->setState(ODriveAxisState::AXIS_STATE_IDLE);
  }

  Serial.println("Emergency Stop!");
}

/**
 * @brief Performs homing sequence using AS5600 absolute encoders
 * 
 * @usage Called once after system initialization or on homing request
 * @note With absolute encoders, homing mainly verifies safe position
 */
// void performHoming() {
//   // 1. Read all AS5600 encoders
//   // 2. Check if within safe startup range
//   // 3. If not, gently move to safe position
//   // 4. Set zero offsets if needed
// }

/**
 * @brief Updates gravity compensation torques based on current arm pose
 * 
 * @usage Called in main control loop (20-100Hz)
 * @note Uses URDF-based calculations or empirical torque mapping
 */
// void updateGravityCompensation() {
//   // 1. Read all joint angles from AS5600 encoders
//   // 2. Calculate required torques for base joints
//   // 3. Apply torques via setTorque()
// }

