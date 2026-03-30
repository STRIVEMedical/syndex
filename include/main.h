#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"

// ── Debug flag ────────────────────────────────────────────────
// Set to 1 to enable serial debug output, 0 to disable
#define DEBUG 1

// Set to 1 for encoder/ODrive read-only bring-up (no motor mode changes, no velocity commands)
#define READ_ONLY_ENCODER_TEST 1

#if DEBUG
  #define DBG(msg)        Serial.println(msg)
  #define DBG_VAL(k, v)   do { Serial.print(k); Serial.println(v);    } while(0)
  #define DBG_FLT(k, v)   do { Serial.print(k); Serial.println(v, 4); } while(0)
#else
  #define DBG(msg)
  #define DBG_VAL(k, v)
  #define DBG_FLT(k, v)
#endif

// ── Print throttle ────────────────────────────────────────────
#define PRINT_INTERVAL_MS 100

void setup();

void computeAngle();

#endif // MAIN_H
