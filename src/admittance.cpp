#include "admittance.h"
#include <math.h>

// Simple single-link gravity model: tau_gravity = m*g*L*cos(theta)
#define ARM_MASS_KG     1.5f     // estimated arm link mass
#define ARM_LENGTH_M    0.3f     // estimated CoM distance from joint
#define GRAVITY         9.81f

void initAdmittance(AdmittanceState* s, float M, float B, float Kt, float ratio) {
    s->M = M;
    s->B = B;
    s->vel = 0.0f;
    s->pos = 0.0f;
    s->torque_constant = Kt;
    s->gear_ratio = ratio;
}

// Estimate gravity torque at the joint given current angle (radians)
static float computeGravityTorque(float angle_rad) {
    return ARM_MASS_KG * GRAVITY * ARM_LENGTH_M * cosf(angle_rad);
}

// Convert motor current → joint torque, subtract gravity → external torque
float estimateExternalTorque(AdmittanceState* s, float iq_measured, float joint_angle) {
    float motor_torque = iq_measured * s->torque_constant * s->gear_ratio;
    float gravity_torque = computeGravityTorque(joint_angle);
    return motor_torque - gravity_torque;
}

// Admittance model: M*a + B*v = F_ext  →  a = (F_ext - B*v) / M
// Integrate with Euler step
void updateAdmittance(AdmittanceState* s, float tau_ext, float dt) {
    float accel = (tau_ext - s->B * s->vel) / s->M;
    s->vel += accel * dt;
    s->pos += s->vel * dt;
}

// Generate a torque command to track the admittance-generated position
// using a simple PD controller around the virtual trajectory
float getAdmittanceTorqueCommand(AdmittanceState* s, float current_pos) {
    float Kp = 5.0f;  // tune
    float Kd = 0.5f;  // tune
    float pos_error = s->pos - current_pos;
    return Kp * pos_error + Kd * s->vel;
}