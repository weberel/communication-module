/*
 * ExtFlash.h  --  driver for the on-board GD25Q256 SPI NOR flash (32 MB, CS = GPIO8).
 * Survives power loss (unlike RTC RAM), so it is the place for a remote logger's
 * store-and-forward buffer or config.
 *
 * NOTE: this uses 3-byte addressing, which reaches the low 16 MB only. The GD25Q256
 * is 32 MB; to use the upper half you need 4-byte addressing (or the extended address
 * register). The low 16 MB is plenty for logging state; extend if you need more.
 *
 * NOR is erase-before-write: a sector (4 KB) must be erased before programming, and
 * you can only program within a 256-byte page per writePage() call.
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "ecotrace_pins.h"

class ExtFlash {
public:
    static constexpr uint32_t SECTOR_SIZE = 4096;
    static constexpr uint32_t PAGE_SIZE   = 256;

    explicit ExtFlash(SPIClass& spi = SPI, uint8_t cs = ECO_PIN_SPI_CS_FLASH,
                      uint32_t freq_hz = 8000000)
        : _spi(&spi), _cs(cs), _freq(freq_hz) {}

    /* Wake from deep power-down and confirm the JEDEC manufacturer is GigaDevice.
     * Assumes SPI has already been started (EcoTrace::beginSPI()). */
    bool begin();
    uint32_t jedecId();          /* mfg<<16 | type<<8 | capacity */

    void read(uint32_t addr, uint8_t* buf, size_t n);
    void eraseSector(uint32_t addr);              /* 4 KB, erases to 0xFF */
    void writePage(uint32_t addr, const uint8_t* buf, size_t n);  /* <=256 B, same page */

    void powerDown();            /* deep power-down (minimise standby current in sleep) */
    void wake();                 /* release from deep power-down */

private:
    SPIClass* _spi;
    uint8_t   _cs;
    uint32_t  _freq;

    void begin_txn() { _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0)); }
    void end_txn()   { _spi->endTransaction(); }
    void addr24(uint32_t a);
    void cmd(uint8_t c);
    uint8_t rdsr();
    void waitReady();
};
