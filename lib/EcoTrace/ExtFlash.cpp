#include "ExtFlash.h"

/* GD25Q256 command set (3-byte addressing) */
#define FCMD_WREN  0x06
#define FCMD_RDSR  0x05
#define FCMD_READ  0x03
#define FCMD_PP    0x02   /* page program */
#define FCMD_SE    0x20   /* 4 KB sector erase */
#define FCMD_RDID  0x9F
#define FCMD_DP    0xB9   /* deep power-down */
#define FCMD_RDP   0xAB   /* release deep power-down */

void ExtFlash::addr24(uint32_t a)
{
    _spi->transfer((a >> 16) & 0xFF);
    _spi->transfer((a >> 8) & 0xFF);
    _spi->transfer(a & 0xFF);
}

void ExtFlash::cmd(uint8_t c)
{
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(c);
    digitalWrite(_cs, HIGH);
    end_txn();
}

uint8_t ExtFlash::rdsr()
{
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(FCMD_RDSR);
    uint8_t s = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);
    end_txn();
    return s;
}

void ExtFlash::waitReady()
{
    uint32_t t0 = millis();
    while ((rdsr() & 0x01) && (millis() - t0) < 2000) delay(1);   /* WIP bit */
}

uint32_t ExtFlash::jedecId()
{
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(FCMD_RDID);
    uint8_t mfg = _spi->transfer(0), type = _spi->transfer(0), cap = _spi->transfer(0);
    digitalWrite(_cs, HIGH);
    end_txn();
    return ((uint32_t)mfg << 16) | ((uint32_t)type << 8) | cap;
}

bool ExtFlash::begin()
{
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    wake();
    return (jedecId() >> 16) == 0xC8;   /* GigaDevice */
}

void ExtFlash::read(uint32_t addr, uint8_t* buf, size_t n)
{
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(FCMD_READ);
    addr24(addr);
    for (size_t i = 0; i < n; i++) buf[i] = _spi->transfer(0x00);
    digitalWrite(_cs, HIGH);
    end_txn();
}

void ExtFlash::eraseSector(uint32_t addr)
{
    cmd(FCMD_WREN);
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(FCMD_SE);
    addr24(addr);
    digitalWrite(_cs, HIGH);
    end_txn();
    waitReady();
}

void ExtFlash::writePage(uint32_t addr, const uint8_t* buf, size_t n)
{
    cmd(FCMD_WREN);
    begin_txn();
    digitalWrite(_cs, LOW);
    _spi->transfer(FCMD_PP);
    addr24(addr);
    for (size_t i = 0; i < n; i++) _spi->transfer(buf[i]);
    digitalWrite(_cs, HIGH);
    end_txn();
    waitReady();
}

void ExtFlash::powerDown() { cmd(FCMD_DP); }
void ExtFlash::wake()      { cmd(FCMD_RDP); delayMicroseconds(50); }
