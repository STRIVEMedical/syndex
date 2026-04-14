#include "admittance_controller.h"

#include <math.h>

#include "admittance.h"
#include "joint.h"

// Conservative defaults to keep first integration stable.
static const float ADM_DEFAULT_M = 0.5f;
static const float ADM_DEFAULT_B = 2.0f;
static const float ADM_DEFAULT_KT = 0.087f;
static const float ADM_DEFAULT_RATIO = 5.0f;

static const float ADM_TAU_DEADBAND_NM = 0.1f;
static const float ADM_MAX_VEL_TURNS_PER_S = 2.0f;

// Controller-owned dynamic state for each logical joint.
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
  // Keep tuned parameters, reset only integrated dynamics.
  for (int i = 0; i < NUM_JOINTS; ++i) {
    g_admittance[i].vel = 0.0f;
    g_admittance[i].pos = 0.0f;
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

  for (int i = 0; i < NUM_JOINTS; ++i) {
    Joint* j = getJoint(i);
    if (j == nullptr || j->odrive == nullptr || j->user_data == nullptr) {
      continue;
    }

    // Start with onboard-encoder ODrive joints only.
    if (!j->use_onboard_encoder) {
      continue;
    }

    // Wait for valid current feedback before applying admittance.
    if (!j->user_data->received_iq_current) {
      continue;
    }

    float angle_rad = j->angle * DEG_TO_RAD;
    float iq_measured = j->user_data->last_iq_msg.Iq_Measured;

    float tau_ext = estimateExternalTorque(&g_admittance[i], iq_measured, angle_rad);
    if (fabsf(tau_ext) < ADM_TAU_DEADBAND_NM) {
      tau_ext = 0.0f;
    }

    updateAdmittance(&g_admittance[i], tau_ext, dt);

    // Convert model rad/s to ODrive turns/s command.
    float vel_cmd = g_admittance[i].vel / (2.0f * PI * g_admittance[i].gear_ratio);
    vel_cmd = constrain(vel_cmd, -ADM_MAX_VEL_TURNS_PER_S, ADM_MAX_VEL_TURNS_PER_S);

    j->odrive->setVelocity(vel_cmd, 0.0f);
  }
}
