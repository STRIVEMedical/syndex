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



static uint32_t last_print_ms = 0;

// ── Admittance state — only allocated for driven joints ───────
AdmittanceState admittance[NUM_JOINTS];
float last_loop_time_us = 0;


// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    DBG("=== SYNDEX BOOT ===");

    // ── ODrive init ───────────────────────────────────────────
    DBG("Initializing ODrives...");
    if (!initMultiOdrives()) {
        Serial.println("[ERROR] ODrive init failed — halting");
        while (1);
    }
    DBG("[OK] All ODrives initialized");

    // ── Peripheral init ───────────────────────────────────────
    Buttons::setup();
    DBG("[OK] Buttons initialized");

    LED::setup();
    ONToggleLED(&LED::powerLED);
    ONToggleLED(&LED::dataLED);
    DBG("[OK] LEDs initialized");

    // ── Joint init ────────────────────────────────────────────
    initJoints();
    DBG("[OK] Joints initialized");

    // Log joint config so we can verify hardware mapping on boot
    #if DEBUG
    Serial.println("── Joint Map ──────────────────────────");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print("  J"); Serial.print(i);
        Serial.print(" ["); Serial.print(joints[i].label); Serial.print("]");
        Serial.print("  odrive=");     Serial.print(joints[i].has_odrive      ? "YES" : "NO ");
        Serial.print("  onboard_enc=");Serial.print(joints[i].use_onboard_encoder ? "YES" : "NO ");
        Serial.print("  i2c_ch=");
        if (joints[i].use_onboard_encoder) {
            Serial.println("N/A");
        } else {
            Serial.println(joints[i].sensor_channel);
        }
    }
    Serial.println("───────────────────────────────────────");
    #endif

    // ── Admittance init — driven joints only ──────────────────
    // Params: M=0.5, B=2.0, Kt=0.087, gear_ratio=5.0
    // Start conservative — raise M if oscillating, lower B if too sluggish
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (joints[i].has_odrive) {
            initAdmittance(&admittance[i], 0.5f, 2.0f, 0.087f, 5.0f);
            DBG_VAL("[OK] Admittance initialized for joint ", i);
        }
    }

    last_loop_time_us = micros();
    DBG("=== ENTERING MAIN LOOP ===\n");
}


// ─────────────────────────────────────────────────────────────
void loop() {

    // ── CAN dispatch — must be first ─────────────────────────
    pumpEvents(can_intf);

    // ── dt calculation ────────────────────────────────────────
    float now = micros();
    float dt  = (now - last_loop_time_us) / 1e6f;
    last_loop_time_us = now;

    if (dt > 0.05f) {
        DBG("[WARN] dt clamped — loop stall detected");
        dt = 0.05f;
    }

    // ── Read all 7 encoders ───────────────────────────────────
    // Joint 0: onboard ODrive encoder via CAN (Pos_Estimate)
    // Joints 1-2: AS5600 via I2C mux
    // Joints 3-6: AS5600 via I2C mux (passive, no ODrive)
    readJointAngles();

    // ── Per-joint admittance loop — driven joints only ────────
    for (int i = 0; i < NUM_JOINTS; i++) {

        // Encoder-only joints — nothing to command, skip
        if (!joints[i].has_odrive) continue;

        ODriveUserData* ud = joints[i].user_data;
        ODriveCAN*      od = joints[i].odrive;

        // Wait until ODrive is sending Iq feedback before running
        if (!ud->received_iq_current) {
            DBG_VAL("[WARN] No Iq feedback yet for joint ", i);
            continue;
        }

        // ── Angle source ──────────────────────────────────────
        // Joint 0: already populated from onboard encoder in readJointAngles()
        // Joints 1-2: populated from AS5600 in readJointAngles()
        // Both sources write into the same joint_angle[] array
        float angle_rad = getJointAngle(i) * DEG_TO_RAD;
        float iq        = ud->last_iq_msg.Iq_Measured;

        // ── Force estimation ──────────────────────────────────
        float tau_ext = estimateExternalTorque(&admittance[i], iq, angle_rad);

        // Deadband — zero out noise below threshold
        bool deadbanded = fabsf(tau_ext) < 0.1f;
        if (deadbanded) tau_ext = 0.0f;

        // ── Admittance model ──────────────────────────────────
        updateAdmittance(&admittance[i], tau_ext, dt);

        // ── Velocity command ──────────────────────────────────
        float vel_cmd = admittance[i].vel / (2.0f * PI * admittance[i].gear_ratio);

        bool clamped = fabsf(vel_cmd) > 2.0f;
        vel_cmd      = constrain(vel_cmd, -2.0f, 2.0f);

        if (clamped) {
            DBG_VAL("[WARN] vel_cmd clamped on joint ", i);
        }

        od->setVelocity(vel_cmd, 0.0f);
    }

    // ── Throttled debug print — all 7 joints ─────────────────
    #if DEBUG
    if ((millis() - last_print_ms) >= PRINT_INTERVAL_MS) {
        Serial.println("─────────────────────────────────────────────");
        Serial.print("dt (ms): "); Serial.println(dt * 1000.0f, 2);

        for (int i = 0; i < NUM_JOINTS; i++) {
            float angle_deg = getJointAngle(i);
            float angle_rad = angle_deg * DEG_TO_RAD;

            Serial.print("J"); Serial.print(i);
            Serial.print(" ["); Serial.print(joints[i].label); Serial.print("]");
            Serial.print("  angle="); Serial.print(angle_deg, 1); Serial.print("deg");

            // Extra info for driven joints only
            if (joints[i].has_odrive && joints[i].user_data->received_iq_current) {
                float iq      = joints[i].user_data->last_iq_msg.Iq_Measured;
                float tau     = estimateExternalTorque(&admittance[i], iq, angle_rad);
                float vel     = admittance[i].vel;
                Serial.print("  Iq=");  Serial.print(iq,  4); Serial.print("A");
                Serial.print("  tau="); Serial.print(tau, 4); Serial.print("Nm");
                Serial.print("  vel="); Serial.print(vel, 4); Serial.print("rad/s");
                Serial.print("  enc="); Serial.print(joints[i].use_onboard_encoder ? "ODRV" : "I2C ");
            } else if (!joints[i].has_odrive) {
                Serial.print("  [encoder only]");
            } else {
                Serial.print("  [waiting for Iq]");
            }
            Serial.println();
        }
        last_print_ms = millis();
    }
    #endif
}