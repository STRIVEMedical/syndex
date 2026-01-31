#ifndef I2C_H
#define I2C_H

#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36
#define ANGLE_HIGH  0x0E
#define ANGLE_LOW   0x0F

extern float zeroOffset[7];   // per-sensor zeroing
extern long turns[7];         // multi-turn tracking
extern int lastRaw[7];


void tcaSelect(uint8_t ch);

uint16_t readRawAS5600();

float computeAngle(int sensorID, uint16_t raw);

void setupI2C();

#endif // I2C_H

