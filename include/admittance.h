#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include <Arduino.h>

// Per-joint admittance state
struct AdmittanceState {
    // Admittance model parameters (tune these)
    float M;        // Virtual mass (kg·m²) — higher = sluggish
    float B;        // Virtual damping (N·m·s/rad) — higher = slower response
    float G;        // Gravity compensation torque (N·m) — function of joint angle

    // State variables
    float vel;      // Virtual velocity (rad/s)
    float pos;      // Virtual position (rad) — tracks desired position

    // Force estimation
    float torque_constant; // Kt of your motor (N·m/A)
    float gear_ratio;      // Pulley/gearbox ratio

    float gravity_torque;  // Current gravity torque estimate
};

void initAdmittance(AdmittanceState* s, float M, float B, float Kt, float ratio);
float estimateExternalTorque(AdmittanceState* s, float iq_measured, float joint_angle);
void updateAdmittance(AdmittanceState* s, float tau_ext, float dt);
float getAdmittanceTorqueCommand(AdmittanceState* s, float current_pos);

#endif