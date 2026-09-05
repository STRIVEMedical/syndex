#ifndef ADMITTANCE_H
#define ADMITTANCE_H
// This will include all of the definitions for functions and variables for the admittance src file
#endif#ifndef ADMITTANCE_H
#define ADMITTANCE_H

// Gear ratio and motor Kt are defined in admittance_controller.cpp as
// ADM_DEFAULT_RATIO and ADM_DEFAULT_KT. Verify both against hardware datasheets
// before tuning the admittance gains.

#include <Arduino.h>

// Per-joint admittance state
struct AdmittanceState {
    // Admittance model parameters (tune these)
    float M;        // Virtual mass (kg·m²) — higher = sluggish
    float B;        // Virtual damping (N·m·s/rad) — higher = slower response

    // State variables
    float vel;      // Virtual velocity (rad/s)
    float pos;      // Virtual position (rad) — tracks desired position

    // Force estimation
    float torque_constant; // Kt of your motor (N·m/A)
    float gear_ratio;      // Pulley/gearbox ratio

    // Baseline current sampled at READY entry with arm stationary.
    // Captures gravity + friction offset so only *changes* from this
    // baseline are treated as human-applied force.
    // Re-calibrated every time resetAdmittanceController() is called.
    float iq_bias;
};

void initAdmittance(AdmittanceState* s, float M, float B, float Kt, float ratio);

float estimateExternalTorque(AdmittanceState* s, float iq_measured, float joint_angle);

void updateAdmittance(AdmittanceState* s, float tau_ext, float dt);

float getAdmittanceTorqueCommand(AdmittanceState* s, float current_pos);

#endif