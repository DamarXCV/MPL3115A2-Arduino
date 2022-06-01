#pragma once

#include <Arduino.h>
#include <Wire.h>

class I2C {
private:
    TwoWire* wire;
    int i2cAddress;

public:
    I2C(TwoWire* wire, int address);
    ~I2C();

    uint8_t whoAmI(uint8_t reg);

    uint8_t read8(uint8_t reg);
    void readBlock(uint8_t* buffer, uint8_t reg, uint8_t size);
    void write8(uint8_t reg, uint8_t val);
    void writeBlock(uint8_t* buffer, uint8_t reg, uint8_t size);
};
