/*
 * MS5837.h  --  driver for the MS5837-02BA pressure / temperature sensor.
 * ----------------------------------------------------------------------
 * TE Connectivity MEMS barometer, 300-1200 mbar absolute, +/-1 mbar typical.
 * Fixed I2C address 0x76 -- there is no address pin, so only one per bus.
 *
 * The part stores six factory calibration coefficients in PROM (CRC-4
 * protected) and returns raw 24-bit ADC readings; all the compensation maths
 * lives here, per the MS5837-02BA01 datasheet.
 *
 * A conversion must be started, then waited for (~20 ms at the highest
 * oversampling) before the result can be read -- read() does both, so it
 * blocks for ~40 ms (pressure + temperature).
 *
 * POWER: on the I2C "hat" header (J401: GND / 3V3 / SCL / SDA) this sensor sits
 * on the ALWAYS-ON 3V3 rail, so it cannot be powered down in deep sleep -- put
 * it in its own low-power state instead (it idles at ~0.1 uA between
 * conversions anyway). Only the separate J404 connector is switched by GPIO14
 * (EcoTrace::sensorRail).
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

class MS5837 {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x76;

    explicit MS5837(TwoWire& bus = Wire, uint8_t addr = DEFAULT_ADDR)
        : _bus(&bus), _addr(addr) {}

    /* Reset, read the calibration PROM and verify its CRC-4. */
    bool begin();

    /* One pressure + temperature conversion (OSR 4096). Blocks ~40 ms.
     * mbar: absolute pressure, degC: temperature. */
    bool read(float& mbar, float& degC);

    /* Approximate altitude from pressure, ISA sea-level model. */
    static float altitudeM(float mbar, float sea_level_mbar = 1013.25f);

    uint16_t coefficient(uint8_t i) const { return (i < 8) ? _c[i] : 0; }
    bool     isPresent();

private:
    TwoWire* _bus;
    uint8_t  _addr;
    uint16_t _c[8] = {0};

    bool     command(uint8_t cmd);
    bool     readPROM();
    bool     convert(uint8_t cmd, uint32_t& value);
    static uint8_t crc4(uint16_t* prom);
};
