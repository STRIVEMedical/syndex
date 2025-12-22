#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36
#define ANGLE_HIGH  0x0E
#define ANGLE_LOW   0x0F

float zeroOffset[2] = {0, 0};   // per-sensor zeroing
long turns[2] = {0, 0};         // multi-turn tracking
int lastRaw[2] = {0, 0};


void tcaSelect(uint8_t ch);

uint16_t readRawAS5600();

float computeAngle(int sensorID, uint16_t raw);

void setupI2C();

void loop();

