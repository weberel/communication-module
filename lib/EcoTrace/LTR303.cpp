#include "LTR303.h"

bool LTR303::rd(uint8_t reg, uint8_t* buf, size_t n)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom((int)_addr, (int)n, (int)true) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = _bus->read();
    return true;
}

bool LTR303::wr(uint8_t reg, uint8_t v)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(v);
    return _bus->endTransmission(true) == 0;
}

bool LTR303::isPresent()
{
    uint8_t pid = 0;
    return rd(0x86, &pid, 1) && (pid & 0xF0) == 0xA0;   /* PART_ID upper nibble 0xA */
}

bool LTR303::begin()
{
    if (!isPresent()) return false;
    setActive(true);
    delay(150);   /* first integration (~100 ms default) */
    return true;
}

void LTR303::setActive(bool on)
{
    wr(0x80, on ? 0x01 : 0x00);   /* ALS_CONTR: bit0 = active mode */
}

bool LTR303::read(uint16_t& ch0, uint16_t& ch1)
{
    uint8_t d[4];
    /* Read order: CH1_0, CH1_1, CH0_0, CH0_1 starting at 0x88. Read CH1 then CH0
     * as the datasheet requires (CH1 low byte read latches the pair). */
    if (!rd(0x88, d, 4)) { ch0 = ch1 = 0; return false; }
    ch1 = ((uint16_t)d[1] << 8) | d[0];
    ch0 = ((uint16_t)d[3] << 8) | d[2];
    return true;
}

uint16_t LTR303::readCh0()
{
    uint16_t ch0 = 0, ch1 = 0;
    read(ch0, ch1);
    return ch0;
}

float LTR303::readLux()
{
    uint16_t ch0, ch1;
    if (!read(ch0, ch1) || (ch0 + ch1) == 0) return 0.0f;
    float ratio = (float)ch1 / (float)(ch0 + ch1);
    /* LTR-303 app note, gain=1x, integration=100 ms (default). */
    if (ratio < 0.45f)      return (1.7743f * ch0 + 1.1059f * ch1);
    else if (ratio < 0.64f) return (4.2785f * ch0 - 1.9548f * ch1);
    else if (ratio < 0.85f) return (0.5926f * ch0 + 0.1185f * ch1);
    return 0.0f;
}
