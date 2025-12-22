#include <Wire.h>
#include "i2c.h"

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
  Wire.write(ANGLE_HIGH);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 2);
  uint8_t high = Wire.read();
  uint8_t low  = Wire.read();
  return (high << 8) | low;
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
  Wire.begin();
  delay(300);

  Serial.println("AS5600 Multi-Sensor Reader Ready");
}

/* Main loop:
   - Listens for a zeroing command ('z') from the host over serial and resets offsets and turn counters
   - Sequentially selects each AS5600 sensor via the TCA9548A mux
   - Reads raw angle data, computes absolute angle with multi-turn tracking
   - Prints device header and both angle values to serial for host parsing
*/
void loop()
{
  // Handle zero request from PC
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      zeroOffset[0] = 0;
      zeroOffset[1] = 0;
      turns[0] = turns[1] = 0;
      Serial.println("Zeroed!");
    }
  }

  // -------- SENSOR 0 --------
  tcaSelect(0);
  delayMicroseconds(500);
  uint16_t raw0 = readRawAS5600();
  float angle0 = computeAngle(0, raw0);

  // -------- SENSOR 1 --------
  tcaSelect(1);
  delayMicroseconds(500);
  uint16_t raw1 = readRawAS5600();
  float angle1 = computeAngle(1, raw1);

  // Output to Python
  Serial.println("I2C READY");
  Serial.print("CH0: ");
  Serial.println(angle0, 2);

  //Prints number of Revolutions for encoder 0
  Serial.print("REV0: ");
  Serial.println(turns[0]);

  Serial.print("CH1: ");
  Serial.println(angle1, 2);

  //Prints number of Revolutions for encoder 1
  Serial.print("REV1: ");
  Serial.println(turns[1]);

  delay(5);
}