#include "MPL3115A2.h"

MPL3115A2::MPL3115A2(int address)
    : I2C(&Wire, address)
{
}

MPL3115A2::MPL3115A2(TwoWire* wire, int address)
    : I2C(wire, address)
{
}

MPL3115A2::~MPL3115A2() { }

boolean MPL3115A2::init()
{
    if (whoAmI(MPL3115A2_REGISTER_WHO_AM_I) != MPL3115A2_WHO_AM_I_VALUE) {
        return false;
    }

    // Software reset
    write8(MPL3115A2_REGISTER_CTRL_REG1, MPL3115A2_CTRL_REG1_BIT_RST);
    while (read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_RST)
        delay(10);

    // set oversampling and altitude mode
    ctrl_reg1.bit.OS = MPL3115A2_CTRL_REG1_OS1;
    ctrl_reg1.bit.ALT = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // enable data ready events for pressure/altitude and temperature
    write8(MPL3115A2_REGISTER_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_BIT_TDEFE | MPL3115A2_PT_DATA_CFG_BIT_PDEFE | MPL3115A2_PT_DATA_CFG_BIT_DREM);

    return true;
}

/**
 * @brief Barometric input for altitude calculations
 *
 * @param sp sea level pressure in Pa
 */
void MPL3115A2::setSeaPressure(uint32_t sp)
{
    delay(10);
    // Register value is input in two Pa units
    uint16_t bar = sp / 2;

    // write result to register
    uint8_t* buffer = new uint8_t[2];
    buffer[0] = bar >> 8;
    buffer[1] = bar & 0xFF;
    writeBlock(buffer, MPL3115A2_REGISTER_BAR_IN_MSB, 2);
    delete[] buffer;
}

uint32_t MPL3115A2::getSeaPressure()
{
    uint8_t* buffer = new uint8_t[2];
    readBlock(buffer, MPL3115A2_REGISTER_BAR_IN_MSB, 2);
    uint32_t value = uint16_t(buffer[0] << 8) | uint16_t(buffer[1]) * 2;
    delete[] buffer;
    return value;
}

void MPL3115A2::setPressureOffset(int16_t offset)
{
    delay(10);
    write8(MPL3115A2_REGISTER_OFF_P, constrain(offset, -512, 508) / 4);
}

int16_t MPL3115A2::getPressureOffset()
{
    return (int8_t(read8(MPL3115A2_REGISTER_OFF_P))) * 4;
}

void MPL3115A2::setAltitudeOffset(int8_t offset)
{
    delay(10);
    write8(MPL3115A2_REGISTER_OFF_H, offset);
}

int8_t MPL3115A2::getAltitudeOffset()
{
    return read8(MPL3115A2_REGISTER_OFF_H);
}

void MPL3115A2::setTemperatureOffset(float offset)
{
    delay(10);
    write8(MPL3115A2_REGISTER_OFF_T, constrain(offset, -8.0f, 7.9375f) / 0.0625f);
}

float MPL3115A2::getTemperatureOffset()
{
    return float(int8_t(read8(MPL3115A2_REGISTER_OFF_T))) * 0.0625f;
}

boolean MPL3115A2::getPressure(float& pressure, float& temp)
{
    size_t i;
    // wait if MPL is messuring
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_OST;
        if (val != 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val != 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // set mode and start messuring
    ctrl_reg1.bit.ALT = 0;
    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // check if data is available
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_DR_STATUS) & MPL3115A2_DR_STATUS_BIT_PTDR;
        if (val == 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val == 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // read data
    uint8_t* buffer = new uint8_t[5];
    readBlock(buffer, MPL3115A2_REGISTER_OUT_P_MSB, 5);
    pressure = float(uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 | uint32_t(buffer[2])) / 64.0f;
    temp = float(uint16_t(buffer[3]) << 8 | uint16_t(buffer[4])) / 256.0f;
    delete[] buffer;
    return true;
}

boolean MPL3115A2::getPressure(float& pressure)
{
    size_t i;
    // wait if MPL is messuring
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_OST;
        if (val != 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val != 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // set mode and start messuring
    ctrl_reg1.bit.ALT = 0;
    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // check if data is available
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_DR_STATUS) & MPL3115A2_DR_STATUS_BIT_PTDR;
        if (val == 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val == 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // read data
    uint8_t* buffer = new uint8_t[5];
    readBlock(buffer, MPL3115A2_REGISTER_OUT_P_MSB, 5);
    pressure = float(uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 | uint32_t(buffer[2])) / 64.0f;
    delete[] buffer;
    return true;
}

boolean MPL3115A2::getAltitude(float& altitude, float& temp)
{
    size_t i;
    // wait if MPL is messuring
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_OST;
        if (val != 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val != 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // set mode and start messuring
    ctrl_reg1.bit.ALT = 1;
    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // check if data is available
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_DR_STATUS) & MPL3115A2_DR_STATUS_BIT_PDR;
        if (val == 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val == 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // read data
    uint8_t* buffer = new uint8_t[5];
    readBlock(buffer, MPL3115A2_REGISTER_OUT_P_MSB, 5);
    altitude = float(uint32_t(buffer[0]) << 24 | uint32_t(buffer[1]) << 16 | uint32_t(buffer[2]) << 8) / 65536.0f;
    temp = float(uint16_t(buffer[3]) << 8 | uint16_t(buffer[4])) / 256.0f;
    delete[] buffer;
    return true;
}

boolean MPL3115A2::getAltitude(float& altitude)
{
    size_t i;
    // wait if MPL is messuring
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_OST;
        if (val != 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val != 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // set mode and start messuring
    ctrl_reg1.bit.ALT = 1;
    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // check if data is available
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_DR_STATUS) & MPL3115A2_DR_STATUS_BIT_PDR;
        if (val == 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val == 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // read data
    uint8_t* buffer = new uint8_t[5];
    readBlock(buffer, MPL3115A2_REGISTER_OUT_P_MSB, 5);
    altitude = float(uint32_t(buffer[0]) << 24 | uint32_t(buffer[1]) << 16 | uint32_t(buffer[2]) << 8) / 655.360f;
    delete[] buffer;
    return true;
}

boolean MPL3115A2::getTemperature(float& temp)
{
    size_t i;
    // wait if MPL is messuring
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_CTRL_REG1) & MPL3115A2_CTRL_REG1_BIT_OST;
        if (val != 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val != 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // start messuring
    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_REGISTER_CTRL_REG1, ctrl_reg1.reg);

    // check if data is available
    for (i = 0; i < MPL3115A2_READ_TRIES; i++) {
        uint8_t val = read8(MPL3115A2_REGISTER_DR_STATUS) & MPL3115A2_DR_STATUS_BIT_TDR;
        if (val == 0 && i == MPL3115A2_READ_TRIES - 1) {
            return false;
        } else if (val == 0) {
            delayMicroseconds(4);
        } else {
            break;
        }
    }

    // read data
    uint8_t* buffer = new uint8_t[2];
    readBlock(buffer, MPL3115A2_REGISTER_OUT_T_MSB, 2);
    temp = float(uint16_t(buffer[0]) << 8 | uint16_t(buffer[1])) / 256.0f;
    delete[] buffer;
    return true;
}
