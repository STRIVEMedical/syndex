
// Key Concepts:
// - ODrive operates in different control modes (position/velocity/torque)
// - This implementation focuses on torque control for gravity compensation
// - AS5600 absolute encoders provide position feedback via I2C
// - CAN bus communicates with ODrive motor controllers

#include <Arduino.h>
#include "odrive.h"
#include "joint.h"

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
  odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
}
  SerialUSB1.println("Closed loop control enabled for Odrive ");
  SerialUSB1.print(node_id);

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
  // Process CAN messages to update status
  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.println("Torque control enabled for Odrive ");
  SerialUSB1.print(node_id);
}

void enable_velocity_control(ODriveCAN &odrv, ODriveUserData &data, uint8_t node_id) {
  // Set control mode to torque control and Set input mode to passthrough (direct torque commands)
  odrv.setControllerMode(
  ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, 
  ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    // Process CAN messages to update status
  for (int i = 0; i < 15; i++) {
    delay(10);
    pumpEvents(can_intf);
  }
  SerialUSB1.println("Velocity control enabled for Odrive ");
  SerialUSB1.print(node_id);
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
  SerialUSB1.print("Waiting for ODrive Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println("...");
  while (!data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  SerialUSB1.print("ODrive Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.print(" Found! Axis State: 0x");
  SerialUSB1.println(data.last_heartbeat.Axis_State, HEX);
  SerialUSB1.println("Entering closed loop control...");
  enable_closed_loop(odrv, data, node_id); 
  // Enable velocity control mode (required for gravity compensation)
  SerialUSB1.println("Enabling velocity control and input passthrough...");
  enable_velocity_control(odrv, data, node_id);
  SerialUSB1.print("ODrive Node ");
  SerialUSB1.print(node_id);
  SerialUSB1.println(" Running!");
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

  // Initialize ODrive 0,1
  if (!initOdrive(odrv0, odrv0_user_data, 0) 
    || !initOdrive(odrv1, odrv1_user_data, 1)
    || !initOdrive(odrv2, odrv2_user_data, 2)) {
    // Serial.println("Odrive init failed!");
    return false;
  }


  SerialUSB1.println("All ODrives Running!");
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
  SerialUSB1.println("Emergency Stop!");
}

/**
 * @brief Performs homing sequence using AS5600 absolute encoders
 * 
 * @usage Called once after system initialization or on homing request
 * @note With absolute encoders, homing mainly verifies safe position
 */
// void performHoming() {
  // 1. Read ODrive positions
  
  // 2. Check if within safe startup range
    // Define a startup range tolerance
  // 3. If not, gently move to safe position
  // 4. Set zero offsets if needed
    // In other words: find starting position of ODrive joints
// }

void performHoming(){
  float home_positions[3] = {0, 0, 0};
  float tolerance = 0.05;
  float homing_velocity = 0.2;
  uint32_t timeout_ms = 5000;

  SerialUSB1.println("Performing homing sequence...");
  Joint* joint_list[3];
  for (int i = 0; i < 3; i++) {
    joint_list[i] = getJoint(i + 4);
  }

  for (int i = 0 ; i <3; i++){
    Joint* j = joint_list[i];

    pumpEvents(can_intf);
    float current_position = j->user_data->last_feedback.Pos_Estimate;
    float error = j->home_angle - current_position;

    SerialUSB1.print("Joint ");
    SerialUSB1.print(j-> label);
    SerialUSB1.print("Position: ");
    SerialUSB1.print(current_position);
    SerialUSB1.print(" (Error: ");
    SerialUSB1.print(error);
    SerialUSB1.println(")");

    if (abs(error) <= tolerance) {
      SerialUSB1.print(j-> label);
      SerialUSB1.println(" already within tolerance, skipping homing.");
      j -> is_homed = true;
      continue;
    }

    SerialUSB1.print("Moving Joint ");
    SerialUSB1.print(j-> label);
    SerialUSB1.println(" to home position...");

    uint32_t start_ms = millis();

    while (true){
      pumpEvents(can_intf);
      current_position = j->user_data->last_feedback.Pos_Estimate;
      error = j->home_angle - current_position;
      if(abs(error) <= tolerance){
        j->odrive->setVelocity(0, 0);
        j->is_homed = true;
        SerialUSB1.print(j-> label);
        SerialUSB1.println(" homed!");
        break;

      }

      if(millis() - start_ms > timeout_ms){
        j->odrive->setVelocity(0, 0);
        SerialUSB1.print(j-> label);
        SerialUSB1.println(" homing timeout!");
        break;
      }

      float vel_cmd = constrain(error * 2, -homing_velocity, homing_velocity);
      j->odrive->setVelocity(vel_cmd, 0);
      delay(10);
    }
  }

  SerialUSB1.println("Homing sequence complete.");

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
    SerialUSB1.print("measured: ");
    SerialUSB1.print(iq_msg.Iq_Measured, 3);
    SerialUSB1.println(" A");
  } else {
    SerialUSB1.print("[IQ] ODrive ");
    SerialUSB1.print(node_id);
    SerialUSB1.println(" getCurrents timeout");
  }
}
