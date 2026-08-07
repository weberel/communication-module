#include "MS5837.h"
#include <math.h>

/* Commands per the MS5837-02BA01 datasheet */
static constexpr uint8_t CMD_RESET   = 0x1E;
static constexpr uint8_t CMD_ADC_READ = 0x00;
static constexpr uint8_t CMD_PROM     = 0xA0;   /* + (word << 1) */
static constexpr uint8_t CMD_CONV_D1  = 0x48;   /* pressure,    OSR 4096 */
static constexpr uint8_t CMD_CONV_D2  = 0x58;   /* temperature, OSR 4096 */
static constexpr uint32_t CONV_MS     = 20;     /* 9.04 ms typ at OSR 4096 */

bool MS5837::command(uint8_t cmd)
{
    _bus->beginTransmission(_addr);
    _bus->write(cmd);
    return _bus->endTransmission() == 0;
}

bool MS5837::isPresent()
{
    _bus->beginTransmission(_addr);
    return _bus->endTransmission() == 0;
}

bool MS5837::readPROM()
{
    for (uint8_t i = 0; i < 7; i++) {
        if (!command((uint8_t)(CMD_PROM + (i << 1)))) return false;
        if (_bus->requestFrom((int)_addr, 2, (int)true) != 2) return false;
        _c[i] = (uint16_t)((_bus->read() << 8) | _bus->read());
    }
    _c[7] = 0;

    uint16_t check[8];
    memcpy(check, _c, sizeof(check));
    uint8_t expected = (uint8_t)((_c[0] >> 12) & 0x0F);
    return crc4(check) == expected;
}

bool MS5837::begin()
{
    if (!command(CMD_RESET)) return false;
    delay(20);                 /* reload of the PROM into the ADC */
    return readPROM();
}

bool MS5837::convert(uint8_t cmd, uint32_t& value)
{
    if (!command(cmd)) return false;
    delay(CONV_MS);
    if (!command(CMD_ADC_READ)) return false;
    if (_bus->requestFrom((int)_addr, 3, (int)true) != 3) return false;
    value = ((uint32_t)_bus->read() << 16) |
            ((uint32_t)_bus->read() << 8)  |
             (uint32_t)_bus->read();
    return true;
}

bool MS5837::read(float& mbar, float& degC)
{
    uint32_t d1 = 0, d2 = 0;
    if (!convert(CMD_CONV_D1, d1)) return false;
    if (!convert(CMD_CONV_D2, d2)) return false;
    if (d1 == 0 || d2 == 0) return false;

    /* First-order compensation (datasheet section "PRESSURE AND TEMPERATURE
     * CALCULATION"); 02BA scaling differs from the 30BA variant. */
    int32_t dT   = (int32_t)d2 - (int32_t)_c[5] * 256;
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _c[6]) / 8388608LL);

    int64_t OFF  = (int64_t)_c[2] * 131072LL + ((int64_t)_c[4] * dT) / 64LL;
    int64_t SENS = (int64_t)_c[1] * 65536LL  + ((int64_t)_c[3] * dT) / 128LL;

    /* Second-order compensation below 20 C */
    int64_t Ti = 0, OFFi = 0, SENSi = 0;
    if (TEMP < 2000) {
        int64_t dt2 = (int64_t)(TEMP - 2000) * (TEMP - 2000);
        Ti    = (11 * (int64_t)dT * dT) / 34359738368LL;
        OFFi  = (31 * dt2) / 8;
        SENSi = (63 * dt2) / 32;
    }
    int64_t OFF2  = OFF - OFFi;
    int64_t SENS2 = SENS - SENSi;
    int32_t TEMP2 = TEMP - (int32_t)Ti;

    int32_t P = (int32_t)((((int64_t)d1 * SENS2) / 2097152LL - OFF2) / 32768LL);

    mbar = (float)P / 100.0f;      /* 02BA reports 0.01 mbar steps */
    degC = (float)TEMP2 / 100.0f;
    return true;
}

float MS5837::altitudeM(float mbar, float sea_level_mbar)
{
    return 44330.0f * (1.0f - powf(mbar / sea_level_mbar, 0.1902949f));
}

/* CRC-4 over the 7 PROM words, per the datasheet reference implementation.
 * Destroys the array it is given. */
uint8_t MS5837::crc4(uint16_t* prom)
{
    uint16_t rem = 0;
    prom[0] &= 0x0FFF;             /* strip the stored CRC */
    prom[7] = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (i & 1) rem ^= (uint16_t)(prom[i >> 1] & 0x00FF);
        else       rem ^= (uint16_t)(prom[i >> 1] >> 8);
        for (uint8_t bit = 8; bit > 0; bit--)
            rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000)
                                 : (uint16_t)(rem << 1);
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}
