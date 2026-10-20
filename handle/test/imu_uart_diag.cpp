// TEMPORARY UART bring-up diagnostic for the BNO085 (delete once imu_test prints readings).
// Works out what the BNO is doing on GPIO16/17: line levels, whether it transmits after an SHTP
// soft reset (in both pin orientations), the raw bytes it sends at 3 Mbaud, UART-RVC frames at
// 115200, and an I2C scan. Ends with a one-line verdict.
//
// For: Handle ESP32-S3

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr int SDA_PIN = 16;  // breakout SDA (BNO TX) per imu.cpp
constexpr int SCL_PIN = 17;  // breakout SCL (BNO RX) per imu.cpp
constexpr int RST_PIN = 4;   // optional: breakout RST (J1-4), used only if a wire is detected
constexpr int INT_PIN = 5;   // optional: breakout INT (J1-5), used only if a wire is detected

const uint8_t softReset[] = {0x7E, 0x01, 0x05, 0x00, 0x01, 0x00, 0x01, 0x7E};

enum Level { HIGH_EXT, FLOATING, LOW_EXT, ODD };

volatile uint32_t edges = 0;
void IRAM_ATTR countEdge() { edges++; }

Level readLevel(int pin) {
    pinMode(pin, INPUT_PULLDOWN);
    delay(5);
    const int down = digitalRead(pin);
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    const int up = digitalRead(pin);
    pinMode(pin, INPUT);

    if (down && up) return HIGH_EXT;
    if (!down && up) return FLOATING;
    if (!down && !up) return LOW_EXT;
    return ODD;
}

const char* levelText(Level l) {
    switch (l) {
        case HIGH_EXT: return "pulled HIGH externally (breakout pull-up present, line idle)";
        case FLOATING: return "FLOATING (nothing attached: open wire or wrong header pin)";
        case LOW_EXT:  return "held LOW externally (BNO holding it, breakout unpowered, or short to GND)";
        default:       return "odd (1 with pulldown, 0 with pullup)";
    }
}

uint32_t passiveEdges(int pin, uint32_t ms) {
    pinMode(pin, INPUT);
    edges = 0;
    attachInterrupt(pin, countEdge, FALLING);
    delay(ms);
    detachInterrupt(pin);
    return edges;
}

void openUart(unsigned long baud, int rxPin, int txPin) {
    Serial1.end();
    Serial1.setRxBufferSize(4096);
    Serial1.begin(baud, SERIAL_8N1, rxPin, txPin);
    while (Serial1.available()) {
        Serial1.read();
    }
}

void sendSoftReset() {
    for (uint8_t b : softReset) {
        Serial1.write(b);
        delay(1);  // BNO needs gaps between host bytes
    }
    Serial1.flush();
}

// Soft-resets the BNO out of txPin and counts falling edges coming back on rxPin.
uint32_t edgesAfterReset(int rxPin, int txPin) {
    openUart(3000000, rxPin, txPin);
    edges = 0;
    attachInterrupt(rxPin, countEdge, FALLING);
    sendSoftReset();
    delay(1000);
    detachInterrupt(rxPin);
    Serial1.end();
    return edges;
}

struct Capture {
    size_t count = 0;
    size_t frameStarts = 0;  // 0x7E followed by protocol ID 0x01
    long firstByteMs = -1;
    uint8_t head[64] = {};
};

// Collects whatever arrives on the open Serial1 for `ms`.
void captureInto(Capture& c, uint32_t ms) {
    const unsigned long start = millis();
    int prev = -1;
    while (millis() - start < ms) {
        while (Serial1.available()) {
            const int b = Serial1.read();
            if (c.firstByteMs < 0) c.firstByteMs = millis() - start;
            if (c.count < sizeof(c.head)) c.head[c.count] = b;
            if (prev == 0x7E && b == 0x01) c.frameStarts++;
            prev = b;
            c.count++;
        }
    }
}

void printCapture(const Capture& c) {
    Serial.printf("  bytes: %u, SHTP frame starts (7E 01): %u, first byte after: %ld ms\n",
                  (unsigned)c.count, (unsigned)c.frameStarts, c.firstByteMs);
    if (c.count > 0) {
        Serial.print("  head:");
        const size_t n = c.count < sizeof(c.head) ? c.count : sizeof(c.head);
        for (size_t i = 0; i < n; i++) {
            Serial.printf(" %02X", c.head[i]);
        }
        Serial.println();
    }
}

Capture captureAfterReset(int rxPin, int txPin) {
    Capture c;
    openUart(3000000, rxPin, txPin);
    sendSoftReset();
    captureInto(c, 1500);
    Serial1.end();
    return c;
}

struct HardReset {
    int sdaInReset = -1;
    int intInReset = -1;
    int sdaAfter = -1;
    int intAfter = -1;
    Capture cap;
};

// Holds the BNO in reset to see whether SDA is still low (a short, not the BNO), then releases it
// with the UART listening, which is the cleanest possible start for UART-SHTP.
HardReset hardResetTest(bool haveInt) {
    HardReset r;
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
    delay(20);
    pinMode(SDA_PIN, INPUT_PULLUP);
    delay(2);
    r.sdaInReset = digitalRead(SDA_PIN);
    if (haveInt) {
        pinMode(INT_PIN, INPUT_PULLUP);
        r.intInReset = digitalRead(INT_PIN);
    }

    openUart(3000000, SDA_PIN, SCL_PIN);
    digitalWrite(RST_PIN, HIGH);
    captureInto(r.cap, 1500);
    Serial1.end();

    pinMode(SDA_PIN, INPUT_PULLUP);
    delay(2);
    r.sdaAfter = digitalRead(SDA_PIN);
    if (haveInt) r.intAfter = digitalRead(INT_PIN);
    return r;
}

// UART-RVC (P0 high) streams 0xAA 0xAA-headed frames at 115200, 100 Hz, without being asked.
size_t rvcHeaders(size_t* bytes) {
    openUart(115200, SDA_PIN, -1);
    size_t headers = 0;
    *bytes = 0;
    int prev = -1;
    const unsigned long start = millis();
    while (millis() - start < 1000) {
        while (Serial1.available()) {
            const int b = Serial1.read();
            (*bytes)++;
            if (prev == 0xAA && b == 0xAA) {
                headers++;
                prev = -1;
                continue;
            }
            prev = b;
        }
    }
    Serial1.end();
    return headers;
}

// A BNO in I2C mode that got clocked mid-byte (e.g. by our UART bytes on SCL) holds SDA low until
// it sees enough clocks: pulse SCL 9 times, then issue a STOP.
bool busRecover(int sda, int scl) {
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, OUTPUT_OPEN_DRAIN);
    for (int i = 0; i < 18; i++) {
        digitalWrite(scl, i & 1);
        delayMicroseconds(10);
    }
    pinMode(sda, OUTPUT_OPEN_DRAIN);
    digitalWrite(sda, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
    digitalWrite(sda, HIGH);
    delayMicroseconds(10);
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delay(2);
    return digitalRead(sda) == HIGH;
}

int i2cFind(int sda, int scl) {
    const bool released = busRecover(sda, scl);
    Serial.printf("  bus recovery SDA=%d SCL=%d: SDA %s\n", sda, scl, released ? "released (HIGH)" : "still LOW");
    Wire.begin(sda, scl, 100000);
    Wire.setTimeOut(20);
    int found = -1;
    const uint8_t addrs[] = {0x4A, 0x4B};
    for (uint8_t a : addrs) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            found = a;
            break;
        }
    }
    Wire.end();
    return found;
}

void runDiag() {
    Serial.println("\n=== BNO085 UART diagnostic ===");

    Serial.println("[1] Idle line levels");
    const Level sda = readLevel(SDA_PIN);
    const Level scl = readLevel(SCL_PIN);
    Serial.printf("  GPIO%d (SDA): %s\n", SDA_PIN, levelText(sda));
    Serial.printf("  GPIO%d (SCL): %s\n", SCL_PIN, levelText(scl));
    const Level rst = readLevel(RST_PIN);
    const Level intr = readLevel(INT_PIN);
    const bool haveRst = rst != FLOATING;
    const bool haveInt = intr != FLOATING;
    Serial.printf("  GPIO%d (RST): %s\n", RST_PIN, haveRst ? levelText(rst) : "not wired");
    Serial.printf("  GPIO%d (INT): %s\n", INT_PIN, haveInt ? levelText(intr) : "not wired");

    Serial.println("[2] Unprompted traffic on GPIO16 (1 s)");
    const uint32_t idleEdges = passiveEdges(SDA_PIN, 1000);
    Serial.printf("  falling edges: %lu\n", (unsigned long)idleEdges);

    Serial.println("[3] Edges after SHTP soft reset @ 3 Mbaud");
    const uint32_t normalEdges = edgesAfterReset(SDA_PIN, SCL_PIN);
    Serial.printf("  expected pins (RX=16, TX=17): %lu edges\n", (unsigned long)normalEdges);
    const uint32_t swappedEdges = edgesAfterReset(SCL_PIN, SDA_PIN);
    Serial.printf("  swapped pins  (RX=17, TX=16): %lu edges\n", (unsigned long)swappedEdges);

    Serial.println("[4] Raw bytes after soft reset @ 3 Mbaud (RX=16, TX=17)");
    const Capture cap = captureAfterReset(SDA_PIN, SCL_PIN);
    printCapture(cap);

    Serial.println("[5] UART-RVC check @ 115200 on GPIO16 (1 s)");
    size_t rvcBytes = 0;
    const size_t rvc = rvcHeaders(&rvcBytes);
    Serial.printf("  bytes: %u, AA AA headers: %u\n", (unsigned)rvcBytes, (unsigned)rvc);

    Serial.println("[6] I2C probe for 0x4A/0x4B");
    int addr = i2cFind(SDA_PIN, SCL_PIN);
    const bool i2cNormal = addr >= 0;
    if (addr < 0) addr = i2cFind(SCL_PIN, SDA_PIN);
    Serial.printf("  %s\n", addr < 0 ? "no answer in either orientation"
                                     : (i2cNormal ? "ANSWERED with SDA=16, SCL=17" : "ANSWERED with SDA=17, SCL=16"));

    HardReset hr;
    if (haveRst) {
        Serial.println("[7] Hard reset via RST (GPIO4), UART listening @ 3 Mbaud");
        hr = hardResetTest(haveInt);
        Serial.printf("  while held in reset: SDA=%s", hr.sdaInReset ? "HIGH" : "LOW");
        if (haveInt) Serial.printf(", INT=%s", hr.intInReset ? "HIGH" : "LOW");
        Serial.println();
        printCapture(hr.cap);
        Serial.printf("  after release: SDA=%s", hr.sdaAfter ? "HIGH" : "LOW");
        if (haveInt) Serial.printf(", INT=%s", hr.intAfter ? "HIGH" : "LOW");
        Serial.println();
    } else {
        Serial.println("[7] Hard reset skipped (no wire on GPIO4 -> RST)");
    }

    Serial.print("\nVERDICT: ");
    if (haveRst && hr.cap.frameStarts > 0) {
        Serial.println("UART-SHTP works after a hard reset via RST. Wiring and strap are good; give imu.cpp the RST pin.");
    } else if (haveRst && hr.sdaInReset == LOW) {
        Serial.println("SDA stays LOW even with the BNO held in reset: the line is shorted (GND/INT) or the breakout is damaged.");
    } else if (cap.frameStarts > 0) {
        Serial.println("BNO answers in UART-SHTP mode on the expected pins. Wiring and strap are good.");
    } else if (rvc > 0) {
        Serial.println("BNO is in UART-RVC mode (P0 high, P1 low). Move the strap wire from P0 to P1, then power-cycle.");
    } else if (addr >= 0) {
        Serial.println("BNO is in I2C mode: it did not see P1 high at power-up. Check P1 reaches 3V3, then power-cycle.");
    } else if (haveRst && hr.sdaInReset == HIGH && hr.sdaAfter == LOW) {
        Serial.println("SDA is fine in reset but the BNO pulls it LOW once it boots: it is not in UART mode (P1 not seen, or P0 high too).");
    } else if (sda == LOW_EXT && scl == LOW_EXT) {
        Serial.println("Both lines held low: breakout looks unpowered (check VIN-GND = 3.3 V) or lines shorted to GND.");
    } else if (sda == LOW_EXT) {
        Serial.println("SDA held low with SCL idle high: BNO stuck in I2C mode (P1 not seen) or SDA shorted to GND.");
    } else if (sda == FLOATING || scl == FLOATING) {
        Serial.println("A line is floating: that GPIO is not connected to the breakout (open joint or wrong header pin).");
    } else if (swappedEdges > 0 && normalEdges == 0) {
        Serial.println("BNO only answers with pins swapped: SDA and SCL wires are reversed.");
    } else if (normalEdges > 0 || cap.count > 0) {
        Serial.println("BNO transmits but bytes don't decode as SHTP at 3 Mbaud: signal integrity (wire length/pull-ups).");
    } else {
        Serial.println("Lines look connected but the BNO never transmits: strap/PS pins, BT/RST pins, or the BNO itself.");
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    runDiag();
}

void loop() {
    delay(1000);
}
