#include "DEBUG.h"
#include "states.h"
#include "LEDs.h"

/**
 * @brief Sets debug LEDs based on current state
 * 
 * Maps state machine states to LED indicators for visual feedback during debugging.
 * 
 * State LED Mappings:
 * - BOOTUP/IDLE: Power LED (waiting for connection)
 * - CONNECTED/HOMING: Data LED (processing)
 * - READY: Power + Data LED (fully operational)
 * - ERROR_STATE: Error LED (fault detected)
 * - POWERINGOFF: All off (shutting down)
 * 
 * @param state Current state machine state
 * @usage Called every cycle in stateUpdate() when DEBUG_MODE is enabled
 */
void setDebugLEDs(state_e state) {
#ifdef DEBUG_MODE
  // Turn off all LEDs first
  OFFToggleLED(&Led::powerLed);
  OFFToggleLED(&Led::dataLed);
  OFFToggleLED(&Led::errorLed);
  
  // Set LEDs based on state
  switch (state) {
    case BOOTUP:
    case IDLE:
      ONToggleLED(&Led::powerLed);  // Waiting for connection
      break;
    case CONNECTED:
    case HOMING:
      ONToggleLED(&Led::dataLed);   // Processing
      break;
    case READY:
      ONToggleLED(&Led::powerLed);  // Fully operational
      ONToggleLED(&Led::dataLed);
      break;
    case ERROR_STATE:
      ONToggleLED(&Led::errorLed);  // Fault detected
      break;
    case POWERINGOFF:
      // All off during shutdown
      break;
    default:
      break;
  }
#endif
}
