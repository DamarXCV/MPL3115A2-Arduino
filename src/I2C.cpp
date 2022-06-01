#include <Arduino.h>
#include <Wire.h>

#include "I2C.h"

I2C::I2C(TwoWire* wire, int address)
    : wire(wire)
    , i2cAddress(address)
{
}

I2C::~I2C() { }

uint8_t I2C::whoAmI(uint8_t reg)
{
    return read8(reg);
}

uint8_t I2C::read8(uint8_t reg)
{
    uint8_t regValue = 0;

    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(i2cAddress, 1);
    if (wire->available()) {
        regValue = wire->read();
    }

    return regValue;
}

void I2C::readBlock(uint8_t* buffer, uint8_t reg, uint8_t size)
{
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    wire->endTransmission(false);
    wire->requestFrom(i2cAddress, size);
    if (wire->available()) {
        for (uint8_t i = 0; i < size; i++) {
            buffer[i] = wire->read();
        }
    }
}

void I2C::write8(uint8_t reg, uint8_t val)
{
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    wire->write(val);
    wire->endTransmission();
}

void I2C::writeBlock(uint8_t* buffer, uint8_t reg, uint8_t size)
{
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    for (uint8_t i = 0; i < size; i++) {
        wire->write(buffer[i]);
    }
    wire->endTransmission();
}
