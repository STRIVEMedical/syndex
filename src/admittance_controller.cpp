#include "admittance_controller.h"

#include <math.h>

#include "admittance.h"
#include "joint.h"
#include "comms.h"   // can_intf for pumpEvents during iq_bias calibration

struct AdmittanceTuning {
  float M;
  float B;
  float Kt;
  float ratio;
  float max_vel_turns_per_s;
};

static const float ADM_DEFAULT_M     = 0.002f;  // virtual inertia — lower = more responsive, higher = smoother
static const float ADM_DEFAULT_B     = 0.03f;  // damping — raise to kill oscillation, lower for more compliance
static const float ADM_DEFAULT_KT    = 0.087f;  // motor Kt (Nm/A)
static const float ADM_DEFAULT_RATIO = 5.0f;    // gearbox reduction

// Per-joint assist tuning. J2/LIFT can be tuned independently for the heavier
// manipulator without changing the other axes.
static const AdmittanceTuning ADM_TUNING[NUM_JOINTS] = {
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, ADM_DEFAULT_RATIO, 4.0f},
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, ADM_DEFAULT_RATIO, 4.0f},
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, ADM_DEFAULT_RATIO, 4.0f},
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, 1.0f,              0.0f},
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, 1.0f,              0.0f},
  {ADM_DEFAULT_M, ADM_DEFAULT_B, ADM_DEFAULT_KT, 1.0f,              0.0f},
};

// In velocity control mode, iq_measured reflects external force (gravity + user push),
// NOT the motor's own command (vel_gain=0.01 draws negligible current).
// This makes iq-based force estimation stable, unlike torque control mode where
// iq_measured includes the commanded current and creates positive feedback.
// Per-joint deadband is now set in joint.cpp (adm_tau_deadband field).
static const float ADM_TAU_SIGN           = -1.0f;  // flip to +1 if arm moves against push
static const uint32_t ADM_DEBUG_INTERVAL_MS = 1000;
static AdmittanceState g_admittance[NUM_JOINTS];

void initAdmittanceController() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    initAdmittance(
        &g_admittance[i],
        ADM_TUNING[i].M,
        ADM_TUNING[i].B,
        ADM_TUNING[i].Kt,
        ADM_TUNING[i].ratio);
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

  static uint32_t lastDebugMs = 0;
  uint32_t nowMs = millis();
  bool doDebug = static_cast<uint32_t>(nowMs - lastDebugMs) >= ADM_DEBUG_INTERVAL_MS;

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
    float max_vel = ADM_TUNING[i].max_vel_turns_per_s;
    vel_cmd = constrain(vel_cmd, -max_vel, max_vel);

    j->odrive->setVelocity(vel_cmd, 0.0f);

    if (doDebug) {
      SerialUSB1.print("[ADMDBG] j"); SerialUSB1.print(i);
      SerialUSB1.print(" iq_bias="); SerialUSB1.print(g_admittance[i].iq_bias, 4);
      SerialUSB1.print(" iq="); SerialUSB1.print(iq_measured, 4);
      SerialUSB1.print(" tau="); SerialUSB1.print(tau_ext, 4);
      SerialUSB1.print(" vel_cmd="); SerialUSB1.println(vel_cmd, 4);
    }

  }

  if (doDebug) {
    lastDebugMs = nowMs;
  }
}
