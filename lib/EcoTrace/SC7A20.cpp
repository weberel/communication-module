#include "SC7A20.h"

bool SC7A20::rd(uint8_t reg, uint8_t* buf, size_t n)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom((int)_addr, (int)n, (int)true) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = _bus->read();
    return true;
}

bool SC7A20::wr(uint8_t reg, uint8_t v)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(v);
    return _bus->endTransmission(true) == 0;
}

bool SC7A20::probe(uint8_t a)
{
    _bus->beginTransmission(a);
    return _bus->endTransmission(true) == 0;
}

bool SC7A20::isPresent()
{
    uint8_t a = probe(ECO_ADDR_SC7A20_LO) ? ECO_ADDR_SC7A20_LO :
                probe(ECO_ADDR_SC7A20_HI) ? ECO_ADDR_SC7A20_HI : 0;
    if (!a) return false;
    _addr = a;
    uint8_t who = 0;
    return rd(0x0F, &who, 1) && who == 0x11;
}

bool SC7A20::begin()
{
    if (!isPresent()) return false;
    if (!wr(0x20, 0x57)) return false;   /* CTRL_REG1: X/Y/Z on, 100 Hz, normal mode */
    delay(20);
    return true;
}

bool SC7A20::readMilliG(int16_t& x, int16_t& y, int16_t& z)
{
    if (!_addr) return false;
    uint8_t d[6];
    if (!rd(0x28 | 0x80, d, 6)) return false;   /* 0x80 = auto-increment */
    x = (int16_t)(((int16_t)((d[1] << 8) | d[0]) >> 6) * 4);   /* 10-bit, 4 mg/LSB @ +/-2g */
    y = (int16_t)(((int16_t)((d[3] << 8) | d[2]) >> 6) * 4);
    z = (int16_t)(((int16_t)((d[5] << 8) | d[4]) >> 6) * 4);
    return true;
}

void SC7A20::powerDown()
{
    if (_addr) wr(0x20, 0x00);
}
