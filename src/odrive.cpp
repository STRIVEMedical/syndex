
// Key Concepts:
// - ODrive operates in different control modes (position/velocity/torque)
// - This implementation focuses on torque control for gravity compensation
// - AS5600 absolute encoders provide position feedback via I2C
// - CAN bus communicates with ODrive motor controllers

#include <Arduino.h>
#include "odrive.h"

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
        ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL) || 
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
  Serial.print("Waiting for ODrive Node ");
  Serial.print(node_id);
  Serial.println("...");
  while (!data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.print("ODrive Node ");
  Serial.print(node_id);
  Serial.print(" Found! Axis State: 0x");
  Serial.println(data.last_heartbeat.Axis_State, HEX);
  // Serial.println("Entering closed loop control...");
  // enable_closed_loop(odrv, data); 
  // Enable torque control mode (required for gravity compensation)
  // Serial.println("Enabling torque control and input passthrough...");
  // enable_torque_control(odrv, data);

  Serial.print("ODrive Node ");
  Serial.print(node_id);
  Serial.println(" Running!");
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
  
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);

  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
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
    || !initOdrive(odrv1, odrv1_user_data, 1)) {
    // Serial.println("Odrive init failed!");
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
    odrive->setTorque(0.0);
  }

  Serial.println("Emergency Stop!");
}

/**
 * @brief Dumps all ODrive configuration and status values
 * 
 * Queries and displays:
 * - Hardware version
 * - Error codes
 * - Bus voltage and current
 * - Temperature
 * - Motor currents
 * - Position and velocity estimates
 * - Power consumption
 * 
 * @usage Call periodically to inspect ODrive state and configuration
 * @note Sends requests over CAN and prints responses
 */
void dumpODriveConfig() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("======================================== ODRIVE STATUS =========================================");
    
    // ODrive 0
    DEBUG_SERIAL.println("\n\t=== ODrive Node 0 ===");
    
    Get_Bus_Voltage_Current_msg_t bus0;
    if (odrv0.getBusVI(bus0, 100)) {
        DEBUG_SERIAL.print("\tBus: ");
        DEBUG_SERIAL.print(bus0.Bus_Voltage, 1);
        DEBUG_SERIAL.print("V @ ");
        DEBUG_SERIAL.print(bus0.Bus_Current, 2);
        DEBUG_SERIAL.println("A");
    }
    
    Get_Temperature_msg_t temp0;
    if (odrv0.getTemperature(temp0, 100)) {
        DEBUG_SERIAL.print("\tTemp: ");
        DEBUG_SERIAL.print(temp0.Temp, 1);
        DEBUG_SERIAL.println("°C");
    }
    
    Get_Iq_msg_t iq0;
    if (odrv0.getCurrents(iq0, 100)) {
        DEBUG_SERIAL.print("\tMotor Current (Iq): ");
        DEBUG_SERIAL.print(iq0.Iq_Setpoint, 2);
        DEBUG_SERIAL.print("A (meas: ");
        DEBUG_SERIAL.print(iq0.Iq_Measured, 2);
        DEBUG_SERIAL.println("A)");
    }
    
    Get_Encoder_Estimates_msg_t fb0;
    if (odrv0.getFeedback(fb0, 100)) {
        DEBUG_SERIAL.print("\tPosition: ");
        DEBUG_SERIAL.print(fb0.Pos_Estimate, 3);
        DEBUG_SERIAL.print(" turns, Velocity: ");
        DEBUG_SERIAL.print(fb0.Vel_Estimate, 2);
        DEBUG_SERIAL.println(" turns/s");
    }
    
    Get_Powers_msg_t pwr0;
    if (odrv0.getPower(pwr0, 100)) {
        DEBUG_SERIAL.print("\tPower: ");
        DEBUG_SERIAL.print(pwr0.Mechanical_Power, 1);
        DEBUG_SERIAL.print("W (elec: ");
        DEBUG_SERIAL.print(pwr0.Electrical_Power, 1);
        DEBUG_SERIAL.println("W)");
    }
    
    // ODrive 1
    DEBUG_SERIAL.println("\n\t=== ODrive Node 1 ===");
    
    Get_Bus_Voltage_Current_msg_t bus1;
    if (odrv1.getBusVI(bus1, 100)) {
        DEBUG_SERIAL.print("\tBus: ");
        DEBUG_SERIAL.print(bus1.Bus_Voltage, 1);
        DEBUG_SERIAL.print("V @ ");
        DEBUG_SERIAL.print(bus1.Bus_Current, 2);
        DEBUG_SERIAL.println("A");
    }
    
    Get_Temperature_msg_t temp1;
    if (odrv1.getTemperature(temp1, 100)) {
        DEBUG_SERIAL.print("\tTemp: ");
        DEBUG_SERIAL.print(temp1.Temp, 1);
        DEBUG_SERIAL.println("°C");
    }
    
    Get_Iq_msg_t iq1;
    if (odrv1.getCurrents(iq1, 100)) {
        DEBUG_SERIAL.print("\tMotor Current (Iq): ");
        DEBUG_SERIAL.print(iq1.Iq_Setpoint, 2);
        DEBUG_SERIAL.print("A (meas: ");
        DEBUG_SERIAL.print(iq1.Iq_Measured, 2);
        DEBUG_SERIAL.println("A)");
    }
    
    Get_Encoder_Estimates_msg_t fb1;
    if (odrv1.getFeedback(fb1, 100)) {
        DEBUG_SERIAL.print("\tPosition: ");
        DEBUG_SERIAL.print(fb1.Pos_Estimate, 3);
        DEBUG_SERIAL.print(" turns, Velocity: ");
        DEBUG_SERIAL.print(fb1.Vel_Estimate, 2);
        DEBUG_SERIAL.println(" turns/s");
    }
    
    Get_Powers_msg_t pwr1;
    if (odrv1.getPower(pwr1, 100)) {
        DEBUG_SERIAL.print("\tPower: ");
        DEBUG_SERIAL.print(pwr1.Mechanical_Power, 1);
        DEBUG_SERIAL.print("W (elec: ");
        DEBUG_SERIAL.print(pwr1.Electrical_Power, 1);
        DEBUG_SERIAL.println("W)");
    }
    
    DEBUG_SERIAL.println("===========================================================================================");
    DEBUG_SERIAL.println();
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

