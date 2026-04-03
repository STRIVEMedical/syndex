#ifndef DEBUG_H
#define DEBUG_H

#include "states.h"

// Uncomment the following line to enable debug mode where LEDs indicate current state
#define DEBUG_MODE

/**
 * @brief Sets debug LEDs based on current state
 * 
 * @param state Current state machine state
 * @usage Called in DEBUG_MODE to visualize state on LEDs
 */
void setDebugLEDs(state_e state);

#endif // DEBUG_H