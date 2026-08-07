#include "flash_log.h"
#include "esp_task_wdt.h"

/* Ring indices survive deep sleep in RTC RAM; s_valid marks them trustworthy.
 * After a power cut (RTC RAM lost) recover() rebuilds them from the chip. */
static RTC_DATA_ATTR uint32_t s_head;
static RTC_DATA_ATTR uint32_t s_tail;
static RTC_DATA_ATTR uint32_t s_valid;
static constexpr uint32_t RTC_VALID_MAGIC = 0xF1A5C3E7;

/* ---- state journal (sector 0) ---- */
typedef struct __attribute__((packed)) {
    uint32_t magic;      /* 'ECST' */
    uint32_t head;
    uint32_t tail;
    uint16_t crc;        /* over the first 12 bytes */
    uint16_t pad;        /* 0xFFFF */
} StateEntry;
static constexpr uint32_t STATE_MAGIC = 0x45435354;
static constexpr uint32_t STATE_ENTRIES = ExtFlash::SECTOR_SIZE / sizeof(StateEntry);

static bool stateValid(const StateEntry& e)
{
    return e.magic == STATE_MAGIC && e.crc == rec_crc16((const uint8_t*)&e, 12);
}

bool FlashLog::begin(bool cold_boot)
{
    _ok = _flash.begin();
    if (!_ok) return false;

    if (!cold_boot && s_valid == RTC_VALID_MAGIC) return true;

    recover();
    s_valid = RTC_VALID_MAGIC;
    return true;
}

void FlashLog::recover()
{
    /* 1. Last valid journal entry -> baseline head/tail. */
    s_head = s_tail = 0;
    for (uint32_t i = 0; i < STATE_ENTRIES; i++) {
        StateEntry e;
        _flash.read(i * sizeof(StateEntry), (uint8_t*)&e, sizeof(e));
        if (stateValid(e)) { s_head = e.head; s_tail = e.tail; }
        else if (e.magic == 0xFFFFFFFF) break;   /* erased -> end of journal */
    }

    /* 2. Forward-scan for records written after the last journal write. Each
     * record carries its own seq, so we advance only over an unbroken chain. */
    for (uint32_t n = 0; n < CAPACITY; n++) {
        if ((n & 4095) == 0) esp_task_wdt_reset();   /* a full scan takes a while */
        LogRecord r;
        _flash.read(slotAddr(s_head), (uint8_t*)&r, sizeof(r));
        if (!rec_valid(&r) || r.seq != s_head) break;
        s_head++;
    }

    if (s_tail > s_head) s_tail = s_head;
    if (s_head - s_tail > CAPACITY - REC_PER_SECTOR)
        s_tail = s_head - (CAPACITY - REC_PER_SECTOR);

    Serial.printf("flash log: recovered head=%lu tail=%lu (%lu pending)\n",
                  (unsigned long)s_head, (unsigned long)s_tail,
                  (unsigned long)(s_head - s_tail));
}

bool FlashLog::append(LogRecord& r)
{
    if (!_ok) return false;

    uint32_t addr = slotAddr(s_head);
    if (addr % ExtFlash::SECTOR_SIZE == 0) {
        /* Entering a fresh sector: erase it, dropping the oldest records if the
         * ring is full (the tail may live in exactly this sector). */
        if (s_head - s_tail > CAPACITY - REC_PER_SECTOR)
            s_tail = s_head - (CAPACITY - REC_PER_SECTOR);
        _flash.eraseSector(addr);
    }

    r.seq = s_head;
    rec_seal(&r);
    _flash.writePage(addr, (const uint8_t*)&r, sizeof(r));
    s_head++;
    return true;
}

uint32_t FlashLog::headSeq() const      { return s_head; }
uint32_t FlashLog::pendingCount() const { return s_head - s_tail; }

bool FlashLog::peek(uint32_t i, LogRecord& r)
{
    if (!_ok || i >= pendingCount()) return false;
    _flash.read(slotAddr(s_tail + i), (uint8_t*)&r, sizeof(r));
    return rec_valid(&r) && r.seq == s_tail + i;
}

void FlashLog::advance(uint32_t n)
{
    s_tail += n;
    if (s_tail > s_head) s_tail = s_head;
    saveState();
}

void FlashLog::saveState()
{
    /* Append the next journal entry; erase the sector when it fills up. */
    uint32_t i = 0;
    for (; i < STATE_ENTRIES; i++) {
        StateEntry e;
        _flash.read(i * sizeof(StateEntry), (uint8_t*)&e, sizeof(e));
        if (e.magic == 0xFFFFFFFF) break;
    }
    if (i >= STATE_ENTRIES) { _flash.eraseSector(0); i = 0; }

    StateEntry e = {};
    e.magic = STATE_MAGIC;
    e.head  = s_head;
    e.tail  = s_tail;
    e.crc   = rec_crc16((const uint8_t*)&e, 12);
    e.pad   = 0xFFFF;
    _flash.writePage(i * sizeof(StateEntry), (const uint8_t*)&e, sizeof(e));
}

void FlashLog::sleep()
{
    if (_ok) _flash.powerDown();
}
