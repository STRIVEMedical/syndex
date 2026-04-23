
// Key Concepts:
// - ODrive operates in different control modes (position/velocity/torque)
// - This implementation focuses on torque control for gravity compensation
// - AS5600 absolute encoders provide position feedback via I2C
// - CAN bus communicates with ODrive motor controllers

#include <Arduino.h>
#include "odrive.h"

// Softer velocity loop tuning for manual admittance testing.
static const float TEST_VEL_GAIN = 0.001f;
static const float TEST_VEL_INT_GAIN = 0.0f;
static const float TEST_VEL_LIMIT_TURNS_PER_S = 50.0f;  // high limit — current_soft_max controls resistance, not this
static const float TEST_CURRENT_SOFT_MAX_A = 6.0f;

static const char* controlModeName(uint8_t mode) {
  switch (mode) {
    case ODriveControlMode::CONTROL_MODE_VOLTAGE_CONTROL: return "VOLTAGE";
    case ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL: return "TORQUE";
    case ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL: return "VELOCITY";
    case ODriveControlMode::CONTROL_MODE_POSITION_CONTROL: return "POSITION";
    default: return "UNKNOWN";
  }
}

static const char* inputModeName(uint8_t mode) {
  switch (mode) {
    case ODriveInputMode::INPUT_MODE_INACTIVE: return "INACTIVE";
    case ODriveInputMode::INPUT_MODE_PASSTHROUGH: return "PASSTHROUGH";
    case ODriveInputMode::INPUT_MODE_VEL_RAMP: return "VEL_RAMP";
    case ODriveInputMode::INPUT_MODE_POS_FILTER: return "POS_FILTER";
    case ODriveInputMode::INPUT_MODE_MIX_CHANNELS: return "MIX_CHANNELS";
    case ODriveInputMode::INPUT_MODE_TRAP_TRAJ: return "TRAP_TRAJ";
    case ODriveInputMode::INPUT_MODE_TORQUE_RAMP: return "TORQUE_RAMP";
    case ODriveInputMode::INPUT_MODE_MIRROR: return "MIRROR";
    case ODriveInputMode::INPUT_MODE_TUNING: return "TUNING";
    default: return "UNKNOWN";
  }
}

static void printLastControllerMode(const ODriveUserData& data, uint8_t node_id) {
  if (!data.received_last_controller_mode || !data.received_last_input_mode) {
    SerialUSB1.print("[ODRIVE] Node ");
    SerialUSB1.print(node_id);
    SerialUSB1.println(" controller mode unknown");
    return;
  }

  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.print(" mode controller=");
  SerialUSB1.print(controlModeName(data.last_controller_mode.Control_Mode));
  SerialUSB1.print(" (0x");
  SerialUSB1.print(data.last_controller_mode.Control_Mode, HEX);
  SerialUSB1.print(") input=");
  SerialUSB1.print(inputModeName(data.last_input_mode.Input_Mode));
  SerialUSB1.print(" (0x");
  SerialUSB1.print(data.last_input_mode.Input_Mode, HEX);
  SerialUSB1.println(")");
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
void enable_closed_loop(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {
  odrv.clearErrors();
  odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" closed loop enabled");
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
void enable_torque_control(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {    
  // Set control mode to torque control and Set input mode to passthrough (direct torque commands)
  odrv.setControllerMode(
  ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
  ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  data.last_controller_mode.Control_Mode = ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL;
  data.last_input_mode.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
  data.received_last_controller_mode = true;
  data.received_last_input_mode = true;
  // Process CAN messages to update status
  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" torque mode enabled");
  printLastControllerMode(data, node_id);
}

void enable_velocity_control(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {
  // Set control mode to torque control and Set input mode to passthrough (direct torque commands)
  odrv.setControllerMode(
  ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, 
  ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  // Reduce loop aggressiveness so joints are easier to backdrive by hand.
  odrv.setVelGains(TEST_VEL_GAIN, TEST_VEL_INT_GAIN);
  odrv.setLimits(TEST_VEL_LIMIT_TURNS_PER_S, TEST_CURRENT_SOFT_MAX_A);
  data.last_controller_mode.Control_Mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;
  data.last_input_mode.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
  data.received_last_controller_mode = true;
  data.received_last_input_mode = true;
    // Process CAN messages to update status
  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" velocity mode enabled");
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.print(" vel_gain=");
  SerialUSB1.print(TEST_VEL_GAIN, 4);
  SerialUSB1.print(" vel_int=");
  SerialUSB1.print(TEST_VEL_INT_GAIN, 4);
  SerialUSB1.print(" vel_limit=");
  SerialUSB1.print(TEST_VEL_LIMIT_TURNS_PER_S, 2);
  SerialUSB1.print(" cur_soft_max=");
  SerialUSB1.println(TEST_CURRENT_SOFT_MAX_A, 2);
  printLastControllerMode(data, node_id);
}

void enable_position_control(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {
  odrv.setControllerMode(
    ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
    ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  data.last_controller_mode.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
  data.last_input_mode.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
  data.received_last_controller_mode = true;
  data.received_last_input_mode = true;
  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" position mode enabled");
}


/* =========================
 * PER-ODRIVE INIT (GENERAL)
 * ========================= */

/**
 * @brief Initializes a single ODrive controller with proper configuration
 * 
 * @param odrv Reference to the ODrive instance to initialize
 * @param data Reference to the ODrive's status data structure
 * @param node_id CAN node ID of this ODrive (0 or 1)
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
bool initOdrive(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {
  // Callbacks already pre-registered in preInitOdriveCallbacks() before CAN started
  
  // Wait for heartbeat to confirm ODrive is online
  SerialUSB1.print("[ODRIVE] Waiting for node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println("...");
  while (!data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.print(" found, axis_state=0x");
  SerialUSB1.println(data.last_heartbeat.Axis_State, HEX);

  // Clear any stale errors from the previous session (e.g. disarm=0x8000 velocity
  // limit violation) so they don't block the closed-loop transition or fail verifyODrive().
  odrv.clearErrors();
  delay(20);
  pumpEvents(can_intf);

  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" entering closed loop...");
  enable_closed_loop(odrv, data, node_id); 
  // Enable velocity control mode (required for gravity compensation)
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" enabling velocity+passthrough...");
  enable_velocity_control(odrv, data, node_id);
  SerialUSB1.print("[ODRIVE] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" running");
  return true;
}

/* =========================
 * PRE-INIT CALLBACKS
 * ========================= */

/**
 * @brief Pre-registers ODrive callbacks before CAN messaging starts
 * 
 * This prevents the "missing callback" error that occurs when heartbeats
 * arrive before callbacks are registered in initOdrive().
 * 
 * @usage Called in setup() before any state machine transitions
 */
void preInitOdriveCallbacks() {
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onCurrents(onCurrents, &odrv0_user_data);
  
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onCurrents(onCurrents, &odrv1_user_data);

  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onCurrents(onCurrents, &odrv2_user_data);

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
  //comms verified in verifyOdrive

  if (!initOdrive(odrv0, odrv0_user_data, 0)
    || !initOdrive(odrv1, odrv1_user_data, 1)
    || !initOdrive(odrv2, odrv2_user_data, 2)) {
    return false;
  }

  SerialUSB1.println("[ODRIVE] All nodes running");
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
    odrive->setTorque(0.0);
  }
  SerialUSB1.println("[SAFETY] Emergency stop asserted");
}


void printOdriveCurrent(ODriveCAN* odrv, ODriveUserData &data, uint8_t node_id) {
  if (odrv == nullptr) return;

  Get_Iq_msg_t iq_msg;
  if (odrv->getCurrents(iq_msg, 20)) {
    data.last_iq_msg = iq_msg;
    data.received_iq_current = true;
    // SerialUSB1.print(iq_msg.Iq_Setpoint, 3);
    SerialUSB1.print("[IQ] ODrive ");
    SerialUSB1.print(node_id);
    SerialUSB1.print(" measured: ");
    SerialUSB1.print(iq_msg.Iq_Measured, 3);
    SerialUSB1.println(" A");
  } else {
    SerialUSB1.print("[IQ] ODrive ");
    SerialUSB1.print(node_id);
    SerialUSB1.println(" getCurrents timeout");
  }
}

bool printOdriveError(ODriveCAN* odrv, uint8_t node_id) {
  if (odrv == nullptr) return false;

  Get_Error_msg_t error_msg;
  if (!odrv->getError(error_msg, 20)) {
    SerialUSB1.print("[ERROR] ODrive ");
    SerialUSB1.print(node_id);
    SerialUSB1.println(" getError timeout");
    return false;
  }

  SerialUSB1.print("[ODRIVE][ERROR] Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.print(" active=0x");
  SerialUSB1.print(error_msg.Active_Errors, HEX);
  SerialUSB1.print(" disarm=0x");
  SerialUSB1.println(error_msg.Disarm_Reason, HEX);

  return error_msg.Active_Errors == 0 && error_msg.Disarm_Reason == 0;
}
