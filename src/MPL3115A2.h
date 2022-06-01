#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "I2C.h"

#define MPL3115A2_READ_TRIES 10

#define MPL3115A2_ADDRESS 0x60

// Register address map
#define MPL3115A2_REGISTER_STATUS 0x00 // Sensor status register
#define MPL3115A2_REGISTER_OUT_P_MSB 0x01 // Pressure data out MSB
#define MPL3115A2_REGISTER_OUT_P_CSB 0x02 // Pressure data out CSB
#define MPL3115A2_REGISTER_OUT_P_LSB 0x03 // Pressure data out LSB
#define MPL3115A2_REGISTER_OUT_T_MSB 0x04 // Temperature data out MSB
#define MPL3115A2_REGISTER_OUT_T_LSB 0x05 // Temperature data out LSB
#define MPL3115A2_REGISTER_DR_STATUS 0x06 // Sensor status register
#define MPL3115A2_REGISTER_OUT_P_DELTA_MSB 0x07 // Pressure data out delta MSB
#define MPL3115A2_REGISTER_OUT_P_DELTA_CSB 0x08 // Pressure data out delta CSB
#define MPL3115A2_REGISTER_OUT_P_DELTA_LSB 0x09 // Pressure data out delta LSB
#define MPL3115A2_REGISTER_OUT_T_DELTA_MSB 0x0A // Temperature data out delta MSB
#define MPL3115A2_REGISTER_OUT_T_DELTA_LSB 0x0B // Temperature data out delta LSB
#define MPL3115A2_REGISTER_WHO_AM_I 0x0C // Device identification register
#define MPL3115A2_REGISTER_F_STATUS 0x0D // FIFO status register
#define MPL3115A2_REGISTER_F_DATA 0x0E // FIFO 8-bit data access
#define MPL3115A2_REGISTER_F_SETUP 0x0F // FIFO setup register
#define MPL3115A2_REGISTER_TIME_DLY 0x10 // Time delay register
#define MPL3115A2_REGISTER_SYSMOD 0x11 // System mode register
#define MPL3115A2_REGISTER_INT_SOURCE 0x12 // Interrupt source register
#define MPL3115A2_REGISTER_PT_DATA_CFG 0x13 // PT data configuration register
#define MPL3115A2_REGISTER_BAR_IN_MSB 0x14 // BAR input in MSB
#define MPL3115A2_REGISTER_BAR_IN_LSB 0x15 // BAR input in LSB
#define MPL3115A2_REGISTER_P_TGT_MSB 0x16 // Pressure target MSB
#define MPL3115A2_REGISTER_P_TGT_LSB 0x17 // Pressure target LSB
#define MPL3115A2_REGISTER_T_TGT 0x18 // Temperature target registeR
#define MPL3115A2_REGISTER_P_WND_MSB 0x19 // Pressure/altitude window MSB
#define MPL3115A2_REGISTER_P_WND_LSB 0x1A // Pressure/altitude window LSB
#define MPL3115A2_REGISTER_T_WND 0x1B // Temperature window registeR
#define MPL3115A2_REGISTER_P_MIN_MSB 0x1C // Minimum pressure data out MSB
#define MPL3115A2_REGISTER_P_MIN_CSB 0x1D // Minimum pressure data out CSB
#define MPL3115A2_REGISTER_P_MIN_LSB 0x1E // Minimum pressure data out LSB
#define MPL3115A2_REGISTER_T_MIN_MSB 0x1F // Minimum temperature data out MSB
#define MPL3115A2_REGISTER_T_MIN_LSB 0x20 // Minimum temperature data out LSB
#define MPL3115A2_REGISTER_P_MAX_MSB 0x21 // Maximum pressure data out MSB
#define MPL3115A2_REGISTER_P_MAX_CSB 0x22 // Maximum pressure data out CSB
#define MPL3115A2_REGISTER_P_MAX_LSB 0x23 // Maximum pressure data out LSB
#define MPL3115A2_REGISTER_T_MAX_MSB 0x24 // Maximum temperature data out MSB
#define MPL3115A2_REGISTER_T_MAX_LSB 0x25 // Maximum temperature data out LSB
#define MPL3115A2_REGISTER_CTRL_REG1 0x26 // Control register 1
#define MPL3115A2_REGISTER_CTRL_REG2 0x27 // Control register 2
#define MPL3115A2_REGISTER_CTRL_REG3 0x28 // Control register 3
#define MPL3115A2_REGISTER_CTRL_REG4 0x29 // Control register 4
#define MPL3115A2_REGISTER_CTRL_REG5 0x2A // Control register 5
#define MPL3115A2_REGISTER_OFF_P 0x2B // Pressure data user offset register
#define MPL3115A2_REGISTER_OFF_T 0x2C // Temperature data user offset register
#define MPL3115A2_REGISTER_OFF_H 0x2D // Altitude data user offset register

// Register DR_STATUS bit description
#define MPL3115A2_DR_STATUS_BIT_TDR 0x02 // New temperature data available
#define MPL3115A2_DR_STATUS_BIT_PDR 0x04 // New pressure/altitude data available
#define MPL3115A2_DR_STATUS_BIT_PTDR 0x08 // New Pressure/altitude or temperature data available
#define MPL3115A2_DR_STATUS_BIT_TOW 0x20 // Temperature data overwrite
#define MPL3115A2_DR_STATUS_BIT_POW 0x40 // Pressure/altitude data overwrite
#define MPL3115A2_DR_STATUS_BIT_PTOW 0x80 // Pressure/altitude or temperature data overwrite

// Register WHO_AM_I
#define MPL3115A2_WHO_AM_I_VALUE 0xC4 // device identifier which is set to C4h by default

// Register F_STATUS bit description
#define MPL3115A2_F_STATUS_BIT_F_OVF 0x80 // FIFO overflow event detected
#define MPL3115A2_F_STATUS_BIT_F_WMRK_FLAG 0x40 // FIFO sample count greater than watermark value
#define MPL3115A2_F_STATUS_SBIT_F_CNT 0x00 // FIFO sample counter. F_CNT[5:0] bits indicate the number of samples currently stored in the FIFO buffer.

// Register F_SETUP bit description
#define MPL3115A2_F_SETUP_SBIT_F_MODE 0x40 // FIFO buffer overflow mode
#define MPL3115A2_F_SETUP_SBIT_F_WMRK 0x00 // FIFO event sample count watermark

// MPL3115A2 F_SETUP F_MODE values
#define MPL3115A2_F_SETUP_BIT_F_MODE_OFF 0x00 // FIFO is disabled (reset value)
#define MPL3115A2_F_SETUP_BIT_F_MODE_CYL 0x40 // FIFO contains the most recent samples when overflowed (circular buffer)
#define MPL3115A2_F_SETUP_BIT_F_MODE_FIL 0x80 // FIFO stops accepting new samples when overflowed
#define MPL3115A2_F_SETUP_BIT_F_MODE_RES0 0xC0 // Not used

// Register SYSMOD bit description
#define MPL3115A2_SYSMOD_BIT_SYSMOD 0x01 // System mode ACTIVE mode

// Register INT_SOURCE bit description
#define MPL3115A2_INT_SOURCE_BIT_SRC_TCHG 0x01 // Delta T interrupt status bit
#define MPL3115A2_INT_SOURCE_BIT_SRC_PCHG 0x02 // Delta P interrupt status bit
#define MPL3115A2_INT_SOURCE_BIT_SRC_TTH 0x04 // Temperature threshold interrupt
#define MPL3115A2_INT_SOURCE_BIT_SRC_PTH 0x08 // Pressure/altitude threshold interrupt
#define MPL3115A2_INT_SOURCE_BIT_SRC_TW 0x10 // Temperature alerter status bit near or equal to target temperature (near is within target value ± window value)
#define MPL3115A2_INT_SOURCE_BIT_SRC_PW 0x20 // Pressure/altitude alerter status bit near or equal to target pressure/altitude (near is within target value ± window value)
#define MPL3115A2_INT_SOURCE_BIT_SRC_FIFO 0x40 // FIFO interrupt status bit
#define MPL3115A2_INT_SOURCE_BIT_SRC_DRDY 0x80 // Data ready interrupt status bit

// Register PT_DATA_CFG bit description
#define MPL3115A2_PT_DATA_CFG_BIT_TDEFE 0x01 // Data event flag enable on new temperature data
#define MPL3115A2_PT_DATA_CFG_BIT_PDEFE 0x02 // Data event flag enable on new pressure/altitude
#define MPL3115A2_PT_DATA_CFG_BIT_DREM 0x04 // Data ready event mode

// Register CTRL_REG1 bit description
#define MPL3115A2_CTRL_REG1_BIT_SBYB 0x01 // This bit sets the mode to ACTIVE
#define MPL3115A2_CTRL_REG1_BIT_OST 0x02 // OST bit will initiate a measurement immediately
#define MPL3115A2_CTRL_REG1_BIT_RST 0x04 // Software reset
#define MPL3115A2_CTRL_REG1_SBIT_OS 0x08 // Oversample ratio
#define MPL3115A2_CTRL_REG1_BIT_ALT 0x80 // Altimeter(1)/barometer(0) mode

// MPL3115A2 CTRL_REG1 OS values
#define MPL3115A2_CTRL_REG1_OS1 0x00
#define MPL3115A2_CTRL_REG1_OS2 0x08
#define MPL3115A2_CTRL_REG1_OS4 0x10
#define MPL3115A2_CTRL_REG1_OS8 0x18
#define MPL3115A2_CTRL_REG1_OS16 0x20
#define MPL3115A2_CTRL_REG1_OS32 0x28
#define MPL3115A2_CTRL_REG1_OS64 0x30
#define MPL3115A2_CTRL_REG1_OS128 0x38

// Register CTRL_REG2 bit description
#define MPL3115A2_CTRL_REG2_SBIT_ST 0x00 // Auto acquisition time step
#define MPL3115A2_CTRL_REG2_BIT_ALARM_SEL 0x10 // The bit selects the target value for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH
#define MPL3115A2_CTRL_REG2_BIT_LOAD_OUTPUT 0x20 // This is to load the target values for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH

// MPL3115A2 CTRL_REG2 ST values (sec)
#define MPL3115A2_CTRL_REG2_ST1 0x00
#define MPL3115A2_CTRL_REG2_ST2 0x01
#define MPL3115A2_CTRL_REG2_ST4 0x02
#define MPL3115A2_CTRL_REG2_ST8 0x03
#define MPL3115A2_CTRL_REG2_ST16 0x04
#define MPL3115A2_CTRL_REG2_ST32 0x05
#define MPL3115A2_CTRL_REG2_ST64 0x06
#define MPL3115A2_CTRL_REG2_ST128 0x07
#define MPL3115A2_CTRL_REG2_ST256 0x08
#define MPL3115A2_CTRL_REG2_ST512 0x09
#define MPL3115A2_CTRL_REG2_ST1024 0x0A
#define MPL3115A2_CTRL_REG2_ST2048 0x0B
#define MPL3115A2_CTRL_REG2_ST4096 0x0C
#define MPL3115A2_CTRL_REG2_ST8192 0x0D
#define MPL3115A2_CTRL_REG2_ST16384 0x0E
#define MPL3115A2_CTRL_REG2_ST32768 0x0F

// Register CTRL_REG3 bit description
#define MPL3115A2_CTRL_REG3_BIT_PP_OD2 0x01 // Push-pull/open drain selection on interrupt pad INT2
#define MPL3115A2_CTRL_REG3_BIT_IPOL2 0x02 // Interrupt polarity active high, or active low on interrupt pad INT2
#define MPL3115A2_CTRL_REG3_BIT_PP_OD1 0x10 // Push-pull/open drain selection on interrupt pad INT1
#define MPL3115A2_CTRL_REG3_BIT_IPOL1 0x20 // Interrupt Polarity active high, or active low on interrupt pad INT1

// Register CTRL_REG4 bit description
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_TCHG 0x01 // Temperature change interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_PCHG 0x02 // Pressure change interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_TTH 0x04 // Temperature threshold interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_PTH 0x08 // Pressure threshold interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_TW 0x10 // Temperature window interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_PW 0x20 // Pressure window interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_FIFO 0x40 // FIFO interrupt enabled
#define MPL3115A2_CTRL_REG4_BIT_INT_EN_DRDY 0x80 // Data ready interrupt enabled

// Register CTRL_REG5 bit description
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_TCHG 0x01 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_PCHG 0x02 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_TTH 0x04 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_PTH 0x08 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_TW 0x10 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_PW 0x20 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_CFG_FIFO 0x40 // Interrupt is routed to INT1
#define MPL3115A2_CTRL_REG5_BIT_INT_EN_DRDY 0x80 // Interrupt is routed to INT1

namespace MPL3115A2 {

class MPL3115A2 : I2C {
private:
    TwoWire* wire;
    int i2cAddress;

    typedef union {
        struct {
            uint8_t SBYB : 1;
            uint8_t OST : 1;
            uint8_t RST : 1;
            uint8_t OS : 3;
            uint8_t RAW : 1;
            uint8_t ALT : 1;
        } bit;
        uint8_t reg;
    } ctrl_reg1_bits;

    ctrl_reg1_bits ctrl_reg1;

public:
    MPL3115A2(int address = MPL3115A2_ADDRESS);
    MPL3115A2(TwoWire* wire, int address = MPL3115A2_ADDRESS);
    ~MPL3115A2();

    boolean init();
    void setSeaPressure(uint32_t sp);
    uint32_t getSeaPressure();

    void setPressureOffset(int16_t offset);
    int16_t getPressureOffset();
    void setAltitudeOffset(int8_t offset);
    int8_t getAltitudeOffset();
    void setTemperatureOffset(float offset);
    float getTemperatureOffset();

    boolean getPressure(float& pressure, float& temp);
    boolean getPressure(float& pressure);
    boolean getAltitude(float& altitude, float& temp);
    boolean getAltitude(float& altitude);
    boolean getTemperature(float& temp);
};

} // namespace MPL3115A2
