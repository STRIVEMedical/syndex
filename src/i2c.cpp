#include <Wire.h>
#include "i2c.h"

// Define globals declared in i2c.h
float zeroOffset[7] = {0, 0, 0, 0, 0, 0, 0};   // per-sensor zeroing
long turns[7] = {0, 0, 0, 0, 0, 0, 0};         // multi-turn tracking
int lastRaw[7] = {0, 0, 0, 0, 0, 0, 0};

/*
Selects channel `ch` on the TCA9548A I2C multiplexer by writing a bitmask 
to its control register (only one channel active at a time)
*/
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/*
Reads the 12-bit raw angle value from the AS5600 encoder by requesting 
two bytes (high and low) from angle registers 0x0E and 0x0F
*/
uint16_t readRawAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);                 // ANGLE register MSB
  Wire.endTransmission(false);      // repeated start

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  return ((msb << 8) | lsb) & 0x0FFF; // 12-bit angle
}

/*
Converts a raw AS5600 value (0–4095) to an absolute angle in degrees,
accounting for full rotation wraps (multi-turn) and applying a zero offset
*/
float computeAngle(int sensorID, uint16_t raw) {

  // Detect forward wrap
  if (raw < 100 && lastRaw[sensorID] > 4000)
    turns[sensorID] += 1;

  // Detect backward wrap
  if (raw > 4000 && lastRaw[sensorID] < 100)
    turns[sensorID] -= 1;

  lastRaw[sensorID] = raw;

  float angle = (raw * 360.0f / 4096.0f) + (turns[sensorID] * 360.0f);

  angle -= zeroOffset[sensorID];
  return angle;
}

/*
Initializes I2C communication and sets up the serial interface
for debugging or streaming encoder data to a host
*/
void setupI2C() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();          // SDA=18, SCL=19
  Wire.setClock(400000);

  // Serial.println("AS5600 via I2C Mux");
}



