/*
 * LTR303.h  --  driver for the LTR-303ALS ambient light sensor (I2C 0x29).
 * Two channels: CH0 = visible + IR, CH1 = IR only. Optional peripheral.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ecotrace_pins.h"

class LTR303 {
public:
    explicit LTR303(TwoWire& bus = Wire, uint8_t addr = ECO_ADDR_LTR303)
        : _bus(&bus), _addr(addr) {}

    /* Verify PART_ID and put the sensor in active mode. Returns false if absent. */
    bool begin();
    bool isPresent();

    void setActive(bool on);      /* ALS_CONTR active(true) / standby(false) */
    void powerDown() { setActive(false); }

    /* Read both channels. Returns false on I2C error. */
    bool read(uint16_t& ch0_vis_ir, uint16_t& ch1_ir);
    uint16_t readCh0();           /* convenience: visible+IR, 0 on error */

    /* Rough lux estimate from the two channels (LTR-303 app-note ratio method,
     * default gain/integration). Good enough for trends, not photometry. */
    float readLux();

private:
    TwoWire* _bus;
    uint8_t  _addr;
    bool rd(uint8_t reg, uint8_t* buf, size_t n);
    bool wr(uint8_t reg, uint8_t v);
};
