#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <Arduino.h>

// Initializes per-joint admittance model state and default gains.
void initAdmittanceController();

// Resets dynamic model state (virtual pos/vel) to avoid jumps on re-entry.
void resetAdmittanceController();

// Runs one READY-state admittance control step.
// dt is loop time in seconds.
void stepAdmittanceController(float dt);

#endif // ADMITTANCE_CONTROLLER_H
