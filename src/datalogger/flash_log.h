/*
 * flash_log.h  --  persistent ring log on the GD25Q128 SPI NOR flash.
 *
 * Layout (16 MB chip, 4 KB sectors):
 *   sector 0        : state journal -- 16 B {head, tail} entries appended one after
 *                     another; the last valid entry wins. Written only on upload
 *                     (2x/day), so the sector is erased every ~4 months and wears out
 *                     never.
 *   sectors 1..4095 : record ring -- 64 B LogRecords, written append-only. When the
 *                     write head enters a fresh sector that sector is erased first,
 *                     silently dropping the oldest records if the ring is full.
 *
 * head = seq of the next record to write, tail = seq of the oldest not-yet-uploaded
 * record. pending = head - tail. Both live in RTC RAM across deep sleep; after a
 * power loss they are recovered from the state journal plus a forward scan (records
 * are self-validating, see record.h), so at most the *upload cursor* since the last
 * upload is at risk -- data is not.
 */
#pragma once

#include <Arduino.h>
#include "ExtFlash.h"
#include "record.h"

class FlashLog {
public:
    static constexpr uint32_t REC_SIZE    = 64;
    static constexpr uint32_t DATA_BASE   = ExtFlash::SECTOR_SIZE;          /* sector 1 */
    static constexpr uint32_t DATA_SECTORS = 4095;                          /* 16 MB - state */
    static constexpr uint32_t REC_PER_SECTOR = ExtFlash::SECTOR_SIZE / REC_SIZE;   /* 64 */
    static constexpr uint32_t CAPACITY    = DATA_SECTORS * REC_PER_SECTOR;  /* 262,080 */

    /* Wake the flash and (on cold boot) recover head/tail from the chip.
     * On a timer wake the RTC-RAM indices are reused and this is cheap. */
    bool begin(bool cold_boot);
    bool ok() const { return _ok; }

    /* Append one record. Fills seq/magic/crc itself. */
    bool append(LogRecord& r);

    uint32_t headSeq() const;
    uint32_t pendingCount() const;

    /* Read the i-th pending record (0 = oldest). False if out of range/corrupt. */
    bool peek(uint32_t i, LogRecord& r);

    /* Mark the oldest n pending records as uploaded and persist the cursor. */
    void advance(uint32_t n);

    /* Deep power-down before sleep (~1 uA -> ~0.1 uA). */
    void sleep();

private:
    ExtFlash _flash;
    bool     _ok = false;

    void recover();
    void saveState();
    uint32_t slotAddr(uint32_t seq) const {
        return DATA_BASE + (seq % CAPACITY) * REC_SIZE;
    }
};
