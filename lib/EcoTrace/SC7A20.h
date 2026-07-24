/*
 * SC7A20.h  --  driver for the SC7A20 3-axis accelerometer (I2C 0x18 / 0x19).
 * Register-compatible with the ST LIS2DH12. Optional peripheral.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ecotrace_pins.h"

class SC7A20 {
public:
    explicit SC7A20(TwoWire& bus = Wire) : _bus(&bus), _addr(0) {}

    /* Auto-detect at 0x18/0x19, check WHO_AM_I, enable X/Y/Z @ 100 Hz. */
    bool begin();
    bool isPresent();
    uint8_t address() const { return _addr; }

    /* Read acceleration in milli-g (+/-2 g range). Returns false on error. */
    bool readMilliG(int16_t& x, int16_t& y, int16_t& z);

    void powerDown();   /* CTRL_REG1 = 0x00 */

private:
    TwoWire* _bus;
    uint8_t  _addr;
    bool rd(uint8_t reg, uint8_t* buf, size_t n);
    bool wr(uint8_t reg, uint8_t v);
    bool probe(uint8_t a);
};
