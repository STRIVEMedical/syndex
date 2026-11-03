// TEMPORARY wiring/mode diagnostic for the BNO085 (delete after debugging).
// GPIO16 read as driven LOW and GPIO17 as pulled HIGH, which looks like a BNO in its default I2C
// mode holding SDA low. Try an I2C bus recovery, scan both pin orientations, and if the BNO answers,
// read orientation over I2C.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static void busRecover(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 18; i++) { digitalWrite(scl, i & 1); delayMicroseconds(10); }
  digitalWrite(scl, HIGH);
  pinMode(scl, INPUT_PULLUP);
  delay(2);
  Serial.printf("after recovery: SDA(GPIO%d)=%d SCL(GPIO%d)=%d\n", sda, digitalRead(sda), scl, digitalRead(scl));
}

static int scan(int sda, int scl) {
  Wire.begin(sda, scl, 100000);
  Wire.setTimeOut(20);
  int hit = -1;
  Serial.printf("I2C scan SDA=%d SCL=%d:", sda, scl);
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); if (a == 0x4A || a == 0x4B) hit = a; }
  }
  Serial.println();
  if (hit < 0) Wire.end();
  return hit;
}

Adafruit_BNO08x bno(-1);
bool running = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BNO085 diag v3 ===");
  int sda = 16, scl = 17;
  busRecover(16, 17);
  int addr = scan(16, 17);
  if (addr < 0) { busRecover(17, 16); addr = scan(17, 16); sda = 17; scl = 16; }
  if (addr < 0) { Serial.println("No BNO085 on I2C in either orientation."); return; }
  Serial.printf("BNO085 found at 0x%02X (SDA=%d SCL=%d) -> it is in I2C mode\n", addr, sda, scl);
  if (!bno.begin_I2C(addr, &Wire)) { Serial.println("begin_I2C failed"); return; }
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 10000)) { Serial.println("enableReport failed"); return; }
  Serial.println("Rotation vector enabled");
  running = true;
}

void loop() {
  if (!running) { delay(1000); return; }
  sh2_SensorValue_t v;
  static unsigned long last = 0;
  if (bno.getSensorEvent(&v) && v.sensorId == SH2_ROTATION_VECTOR && millis() - last >= 100) {
    last = millis();
    float w = v.un.rotationVector.real, x = v.un.rotationVector.i, y = v.un.rotationVector.j, z = v.un.rotationVector.k;
    float roll  = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 57.2958f;
    float pitch = asinf(constrain(2 * (w * y - z * x), -1.0f, 1.0f)) * 57.2958f;
    float yaw   = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 57.2958f;
    Serial.printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n", roll, pitch, yaw);
  }
}
