#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "odrive.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "main.h"
#include "admittance.h"

// ── Debug flag ────────────────────────────────────────────────
// Set to 1 to enable serial debug output, 0 to disable
#define DEBUG 1

#if DEBUG
  #define DBG(msg)        Serial.println(msg)
  #define DBG_VAL(k, v)   do { Serial.print(k); Serial.println(v); } while(0)
  #define DBG_FLT(k, v)   do { Serial.print(k); Serial.println(v, 4); } while(0)
#else
  #define DBG(msg)
  #define DBG_VAL(k, v)
  #define DBG_FLT(k, v)
#endif

// ── Print throttle ────────────────────────────────────────────
// Prints debug every N ms so serial doesn't flood
#define PRINT_INTERVAL_MS 100
static uint32_t last_print_ms = 0;

AdmittanceState admittance[NUM_JOINTS];
float last_loop_time_us = 0;


// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500); // give serial monitor time to connect
    DBG("=== SYNDEX BOOT ===");

    DBG("Initializing ODrives...");
    if (!initMultiOdrives()) {
        Serial.println("[ERROR] ODrive init failed — halting");
        while (1);
    }
    DBG("[OK] ODrives initialized");

    Buttons::setup();
    DBG("[OK] Buttons initialized");

    LED::setup();
    ONToggleLED(&LED::powerLED);
    ONToggleLED(&LED::dataLED);
    DBG("[OK] LEDs initialized");

    initJoints();
    DBG("[OK] Joints initialized");

    // Admittance params: M=0.5, B=2.0, Kt=0.087, gear_ratio=5.0
    // Start conservative — increase M if oscillating, lower B if too sluggish
    for (int i = 0; i < NUM_JOINTS; i++) {
        initAdmittance(&admittance[i], 0.5f, 2.0f, 0.087f, 5.0f);
        DBG_VAL("[OK] Admittance initialized for joint ", i);
    }

    last_loop_time_us = micros();
    DBG("=== ENTERING MAIN LOOP ===");
}


// ─────────────────────────────────────────────────────────────
void loop() {
    pumpEvents(can_intf);

    // ── dt calculation ──────────────────────────────────────
    float now = micros();
    float dt  = (now - last_loop_time_us) / 1e6f;
    last_loop_time_us = now;

    // Clamp dt — prevents a huge integration step if loop stalls
    if (dt > 0.05f) {
        DBG("[WARN] dt clamped — loop stall detected");
        dt = 0.05f;
    }

    // ── Sensor read ─────────────────────────────────────────
    readJointAngles();

    // ── Per-joint admittance loop ───────────────────────────
    for (int i = 0; i < NUM_JOINTS; i++) {
        ODriveUserData* ud = joints[i].user_data;
        ODriveCAN*      od = joints[i].odrive;

        // Skip joint if no current feedback received yet
        if (!ud->received_iq_current) {
            DBG_VAL("[WARN] No Iq feedback yet for joint ", i);
            continue;
        }

        float angle_rad = getJointAngle(i) * DEG_TO_RAD;
        float iq        = ud->last_iq_msg.Iq_Measured;

        // ── Force estimation ─────────────────────────────
        float tau_ext = estimateExternalTorque(&admittance[i], iq, angle_rad);

        // Deadband — zero out noise below threshold
        bool deadbanded = fabsf(tau_ext) < 0.1f;
        if (deadbanded) tau_ext = 0.0f;

        // ── Admittance model update ───────────────────────
        updateAdmittance(&admittance[i], tau_ext, dt);

        // ── Velocity command ──────────────────────────────
        float vel_cmd = admittance[i].vel / (2.0f * PI * admittance[i].gear_ratio);

        // Clamp velocity to safe range before sending
        bool clamped  = fabsf(vel_cmd) > 2.0f;
        vel_cmd       = constrain(vel_cmd, -2.0f, 2.0f);

        if (clamped) {
            DBG_VAL("[WARN] vel_cmd clamped on joint ", i);
        }

        od->setVelocity(vel_cmd, 0.0f);

        // ── Throttled debug print ─────────────────────────
        #if DEBUG
        if ((millis() - last_print_ms) >= PRINT_INTERVAL_MS && i == 0) {
            Serial.println("─────────────────────────");
            Serial.print  ("dt (ms)     : "); Serial.println(dt * 1000.0f, 2);

            for (int j = 0; j < NUM_JOINTS; j++) {
                float a_rad  = getJointAngle(j) * DEG_TO_RAD;
                float iq_j   = joints[j].user_data->last_iq_msg.Iq_Measured;
                float tau_j  = estimateExternalTorque(&admittance[j], iq_j, a_rad);
                float vel_j  = admittance[j].vel;

                Serial.print("J"); Serial.print(j);
                Serial.print(" ["); Serial.print(joints[j].label); Serial.print("]");
                Serial.print("  angle=");  Serial.print(a_rad * RAD_TO_DEG, 1);
                Serial.print("deg  Iq=");  Serial.print(iq_j, 4);
                Serial.print("A  tau=");   Serial.print(tau_j, 4);
                Serial.print("Nm  vel=");  Serial.print(vel_j, 4);
                Serial.print("rad/s  db=");Serial.println(deadbanded ? "Y" : "N");
            }
            last_print_ms = millis();
        }
        #endif
    }
}