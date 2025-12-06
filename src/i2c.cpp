// #include <Arduino.h>
#include <Wire.h>
#include "TCA9548.h"

/*
Using I2C #1:
SCL: Pin 18 -- A4
SDA: Pin 19 -- A5
*/

#define I2C_BUS Wire1           // Use I2C1
#define TCA_ADDR 0x70           // Default I2C address of TCA9548A

TCA9548 MP(0x70);
uint8_t channels = 0;


// Helper: Select TCA9548A channel (0–7)
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    I2C_BUS.beginTransmission(TCA_ADDR);
    I2C_BUS.write(1 << channel);
    I2C_BUS.endTransmission();
}

// Example: Read 1 byte from a device on a selected channel
uint8_t readFromDevice(uint8_t channel, uint8_t deviceAddr, uint8_t registerAddr) {
    tcaSelect(channel);  // Select MUX channel
    I2C_BUS.beginTransmission(deviceAddr);
    I2C_BUS.write(registerAddr);
    I2C_BUS.endTransmission(false);  // Restart for read

    I2C_BUS.requestFrom(deviceAddr, (uint8_t)1);
    if (I2C_BUS.available()) {
        return I2C_BUS.read();
    }
    return 0xFF; // Indicate error
}

void setup() {
    Serial.begin(115200);
    // I2C_BUS.begin();  // Initialize I2C1

    Serial.begin(115200);
    Serial.println();
    Serial.println(__FILE__);
    Serial.print("TCA9548_LIB_VERSION: ");
    Serial.println(TCA9548_LIB_VERSION);
    Serial.println();

    Wire.begin();
    if (MP.begin() == false)
    {
        Serial.println("COULD NOT CONNECT TO MULTIPLEXER");
    }

    channels = MP.channelCount();
    Serial.print("CHAN:\t");
    Serial.println(MP.channelCount());

    //  adjust address range to your needs.
    for (uint8_t addr = 60; addr < 70; addr++)
    {
        if (addr % 10 == 0) Serial.println();
        Serial.print(addr);
        Serial.print("\t");
        Serial.print(MP.find(addr), BIN);
        Serial.println();
    }

    Serial.println("done...");


    // uint8_t channel = 2;
    // uint8_t devAddr = 0x68;       // Example: IMU or RTC
    // uint8_t regAddr = 0x75;       // Example: WHO_AM_I register for MPU6050

    // uint8_t data = readFromDevice(channel, devAddr, regAddr);
    // Serial.printf("Read 0x%02X from device 0x%02X on channel %u\n", data, devAddr, channel);
}

void loop() {
    // Empty or polling logic
}

int main() {
    setup();

    while (1) {
        
    }
    return 0;
}