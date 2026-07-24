#include "ATECC608B.h"

bool ATECC608B::probe(uint8_t a)
{
    _bus->beginTransmission(a);
    return _bus->endTransmission(true) == 0;
}

void ATECC608B::wake()
{
    _bus->beginTransmission(0x00);
    _bus->endTransmission(true);
    delay(2);   /* tWHI */
}

bool ATECC608B::begin()
{
    wake();
    if (probe(ECO_ADDR_ATECC608B_A))      _addr = ECO_ADDR_ATECC608B_A;
    else { wake(); if (probe(ECO_ADDR_ATECC608B_B)) _addr = ECO_ADDR_ATECC608B_B; }
    return _addr != 0;
}

void ATECC608B::sleep()
{
    if (!_addr) return;
    _bus->beginTransmission(_addr);
    _bus->write(0x01);   /* Sleep word address */
    _bus->endTransmission(true);
}
