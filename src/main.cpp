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
    // Joints with ODrive: onboard ODrive encoder via CAN (Pos_Estimate)
    // Joints without ODrive: AS5600 via I2C mux
    readJointAngles();

    // ── Per-joint admittance loop — driven joints only ────────
    #if !READ_ONLY_ENCODER_TEST
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
        // Driven joints use onboard ODrive encoder values from readJointAngles()
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
    #endif

    // ── Throttled debug print — all 7 joints ─────────────────
    #if DEBUG
    if ((millis() - last_print_ms) >= PRINT_INTERVAL_MS) {
        Serial.println("──────── Encoder Snapshot ────────");
        Serial.print("dt(ms)=");
        Serial.println(dt * 1000.0f, 2);

        // Print configured ODrive joints side-by-side on one line.
        int odrive_indices[2] = {-1, -1};
        int odrive_count = 0;
        for (int i = 0; i < NUM_JOINTS && odrive_count < 2; i++) {
            if (joints[i].has_odrive && joints[i].use_onboard_encoder) {
                odrive_indices[odrive_count++] = i;
            }
        }

        if (odrive_count == 0) {
            Serial.println("ODRIVE [N/A] angle=N/A");
        } else {
            Serial.print("ODRIVES | ");
            for (int k = 0; k < odrive_count; k++) {
                int idx = odrive_indices[k];
                Serial.print("[");
                Serial.print(joints[idx].label);
                Serial.print("] ");
                Serial.print(getJointAngle(idx), 2);
                Serial.print(" deg");
                if (k < odrive_count - 1) {
                    Serial.print("  ||  ");
                }
            }
            Serial.println();
        }
        last_print_ms = millis();
    }
    #endif
}