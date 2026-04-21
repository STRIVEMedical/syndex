#include "admittance_controller.h"

#include <math.h>

#include "admittance.h"
#include "joint.h"
#include "comms.h"   // can_intf for pumpEvents during iq_bias calibration

// Conservative defaults to keep first integration stable.
static const float ADM_DEFAULT_M = 0.03f;
static const float ADM_DEFAULT_B = 0.05f;
static const float ADM_DEFAULT_KT = 0.087f;
static const float ADM_DEFAULT_RATIO = 5.0f;

static const float ADM_TAU_DEADBAND_NM = 0.10f;   // raised: filters gravity residuals + noise
static const float ADM_MAX_VEL_TURNS_PER_S = 1.0f; // lowered: prevents runaway during tuning
static const float ADM_TAU_SIGN = -1.0f;            // flip to +1 if arm moves against your push
static const uint32_t ADM_DEBUG_INTERVAL_MS = 100;

// Controller-owned dynamic state for each logical joint.
static AdmittanceState g_admittance[NUM_JOINTS];
static uint32_t g_last_adm_debug_ms = 0;

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
  // Keep tuned parameters, reset integrated dynamics, calibrate iq_bias.
  for (int i = 0; i < NUM_JOINTS; ++i) {
    g_admittance[i].vel = 0.0f;
    g_admittance[i].pos = 0.0f;

    // Sample iq 10 times to measure the baseline current (gravity + friction
    // at this arm position). Only deviations from this bias are treated as
    // human-applied force. Re-run resetAdmittanceController() any time the
    // arm moves to a new resting position and you want to recalibrate.
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
    // SerialUSB1.print("[ADM] j");
    // SerialUSB1.print(i);
    // SerialUSB1.print(" iq_bias=");
    // SerialUSB1.print(g_admittance[i].iq_bias, 4);
    // SerialUSB1.println(" A");
  }
}

void stepAdmittanceController(float dt) {
  // Clamp dt to avoid large integration jumps after stalls.
  if (dt <= 0.0f) {
    return;
  }
  if (dt > 0.05f) {
    dt = 0.05f;
  }

  uint32_t now_ms = millis();
  bool should_debug = (now_ms - g_last_adm_debug_ms) >= ADM_DEBUG_INTERVAL_MS;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    Joint* j = getJoint(i);
    if (j == nullptr || j->odrive == nullptr || j->user_data == nullptr) {
      continue;
    }

    // Start with onboard-encoder ODrive joints only.
    if (!j->use_onboard_encoder) {
      continue;
    }

    // Poll fresh iq current each cycle; fall back to last valid sample.
    Get_Iq_msg_t iq_msg;
    if (j->odrive->getCurrents(iq_msg, 3)) {
      j->user_data->last_iq_msg = iq_msg;
      j->user_data->received_iq_current = true;
    }

    if (!j->user_data->received_iq_current) {
      continue;
    }

    float angle_rad = j->angle * DEG_TO_RAD;
    float iq_measured = j->user_data->last_iq_msg.Iq_Measured;

    float tau_raw = estimateExternalTorque(&g_admittance[i], iq_measured, angle_rad);
    float tau_ext = ADM_TAU_SIGN * tau_raw;
    if (fabsf(tau_ext) < ADM_TAU_DEADBAND_NM) {
      tau_ext = 0.0f;
    }

    updateAdmittance(&g_admittance[i], tau_ext, dt);

    // Convert model rad/s to ODrive turns/s command.
    float vel_cmd = g_admittance[i].vel / (2.0f * PI * g_admittance[i].gear_ratio);
    vel_cmd = constrain(vel_cmd, -ADM_MAX_VEL_TURNS_PER_S, ADM_MAX_VEL_TURNS_PER_S);

    j->odrive->setVelocity(vel_cmd, 0.0f);

    // if (should_debug) {
    //   SerialUSB1.print("[ADM] j");
    //   SerialUSB1.print(i);
    //   SerialUSB1.print(" iq=");
    //   SerialUSB1.print(iq_measured, 3);
    //   SerialUSB1.print("A (bias=");
    //   SerialUSB1.print(g_admittance[i].iq_bias, 3);
    //   SerialUSB1.print(") tau_raw=");
    //   SerialUSB1.print(tau_raw, 3);
    //   SerialUSB1.print("Nm tau_ext=");
    //   SerialUSB1.print(tau_ext, 3);
    //   SerialUSB1.print("Nm vel_cmd=");
    //   SerialUSB1.print(vel_cmd, 3);
    //   SerialUSB1.println(" turns/s");
    // }
  }

  if (should_debug) {
    g_last_adm_debug_ms = now_ms;
  }
}
