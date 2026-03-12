#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "buttons.h"
#include "LEDs.h"
#include "joint.h"

void setup();

void computeAngle();

#endif // MAIN_H
