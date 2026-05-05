#include "admittance_controller.h"

#include <math.h>

#include "admittance.h"
#include "joint.h"
#include "comms.h"   // can_intf for pumpEvents during iq_bias calibration

static const float ADM_DEFAULT_M     = 0.004f;  // virtual inertia — lower = more responsive, higher = smoother
static const float ADM_DEFAULT_B     = 0.08f;  // damping — raise to kill oscillation, lower for more compliance
static const float ADM_DEFAULT_KT    = 0.087f;  // motor Kt (Nm/A)
static const float ADM_DEFAULT_RATIO = 5.0f;    // gearbox reduction

// In velocity control mode, iq_measured reflects external force (gravity + user push),
// NOT the motor's own command (vel_gain=0.01 draws negligible current).
// This makes iq-based force estimation stable, unlike torque control mode where
// iq_measured includes the commanded current and creates positive feedback.
// Per-joint deadband is now set in joint.cpp (adm_tau_deadband field).
static const float ADM_MAX_VEL_TURNS_PER_S = 4.0f;
static const float ADM_TAU_SIGN           = -1.0f;  // flip to +1 if arm moves against push
static AdmittanceState g_admittance[NUM_JOINTS];

void initAdmittanceController() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    initAdmittance(
        &g_admittance[i],
        ADM_DEFAULT_M,
        ADM_DEFAULT_B,
        ADM_DEFAULT_KT,
        ADM_DEFAULT_RATIO);
  }
}

void resetAdmittanceController() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    g_admittance[i].vel = 0.0f;
    g_admittance[i].pos = 0.0f;

    // Sample iq 10 times to calibrate the gravity+friction baseline.
    // Only deviations from this bias are treated as human-applied force.
    // Re-calibrates every time the system enters READY.
    Joint* j = getJoint(i);
    if (j == nullptr || j->odrive == nullptr || j->user_data == nullptr
        || !j->use_onboard_encoder) {
      continue;
    }

    float sum = 0.0f;
    int count = 0;
    for (int k = 0; k < 10; ++k) {
      pumpEvents(can_intf);
      Get_Iq_msg_t iq_msg;
      if (j->odrive->getCurrents(iq_msg, 5)) {
        sum += iq_msg.Iq_Measured;
        ++count;
      }
      delay(5);
    }

    g_admittance[i].iq_bias = (count > 0) ? (sum / count) : 0.0f;
    SerialUSB1.print("[ADM] j"); SerialUSB1.print(i);
    SerialUSB1.print(" iq_bias="); SerialUSB1.print(g_admittance[i].iq_bias, 4);
    SerialUSB1.println(" A");
  }
}

void stepAdmittanceController(float dt) {
  if (dt <= 0.0f) return;
  if (dt > 0.05f) dt = 0.05f;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    Joint* j = getJoint(i);
    if (j == nullptr || j->odrive == nullptr || j->user_data == nullptr) continue;
    if (!j->use_onboard_encoder) continue;

    // Poll fresh iq current; fall back to last valid sample.
    Get_Iq_msg_t iq_msg;
    if (j->odrive->getCurrents(iq_msg, 3)) {
      j->user_data->last_iq_msg = iq_msg;
      j->user_data->received_iq_current = true;
    }
    if (!j->user_data->received_iq_current) continue;

    float angle_rad   = j->angle * DEG_TO_RAD;
    float iq_measured = j->user_data->last_iq_msg.Iq_Measured;

    // Estimate external (human-applied) torque. In velocity control mode the motor
    // draws almost no current itself (vel_gain=0.01), so iq_measured ≈ external torque.
    float tau_raw = estimateExternalTorque(&g_admittance[i], iq_measured, angle_rad);
    float tau_ext = ADM_TAU_SIGN * tau_raw;
    if (fabsf(tau_ext) < j->adm_tau_deadband) tau_ext = 0.0f;

    updateAdmittance(&g_admittance[i], tau_ext, dt);

    // Convert joint rad/s → motor turns/s: motor spins gear_ratio× faster than joint.
    float vel_cmd = g_admittance[i].vel * j->gear_ratio / (2.0f * PI);
    vel_cmd = constrain(vel_cmd, -ADM_MAX_VEL_TURNS_PER_S, ADM_MAX_VEL_TURNS_PER_S);

    j->odrive->setVelocity(vel_cmd, 0.0f);

  }

}
