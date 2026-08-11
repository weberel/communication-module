#include "flash_log.h"
#include "ext_flash.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "flashlog";

/* Ring cursors survive deep sleep in RTC RAM; magic marks them trustworthy.
 * After a power cut recover() rebuilds them from the chip. */
static RTC_DATA_ATTR uint32_t s_head;
static RTC_DATA_ATTR uint32_t s_tail;
static RTC_DATA_ATTR uint32_t s_valid;
#define RTC_VALID_MAGIC 0xF1A5C3E7u

static bool s_ok;

/* ---- state journal (sector 0), format shared with the Arduino build ---- */
typedef struct __attribute__((packed)) {
    uint32_t magic;      /* 'ECST' */
    uint32_t head;
    uint32_t tail;
    uint16_t crc;        /* over the first 12 bytes */
    uint16_t pad;        /* 0xFFFF */
} StateEntry;
#define STATE_MAGIC   0x45435354u
#define STATE_ENTRIES (EXTFLASH_SECTOR_SIZE / sizeof(StateEntry))

static uint32_t slot_addr(uint32_t seq)
{
    return FLASHLOG_DATA_BASE + (seq % FLASHLOG_CAPACITY) * FLASHLOG_REC_SIZE;
}

static bool state_valid(const StateEntry *e)
{
    return e->magic == STATE_MAGIC &&
           e->crc == rec_crc16((const uint8_t *)e, 12);
}

static void recover(void)
{
    /* 1. Last valid journal entry -> baseline cursors. */
    s_head = s_tail = 0;
    for (uint32_t i = 0; i < STATE_ENTRIES; i++) {
        StateEntry e;
        extflash_read(i * sizeof(StateEntry), (uint8_t *)&e, sizeof(e));
        if (state_valid(&e)) { s_head = e.head; s_tail = e.tail; }
        else if (e.magic == 0xFFFFFFFFu) break;   /* erased -> end of journal */
    }

    /* 2. Forward-scan for records written after the last journal write. */
    for (uint32_t n = 0; n < FLASHLOG_CAPACITY; n++) {
        if ((n & 4095) == 0) esp_task_wdt_reset();
        LogRecord r;
        extflash_read(slot_addr(s_head), (uint8_t *)&r, sizeof(r));
        if (!rec_valid(&r) || r.seq != s_head) break;
        s_head++;
    }

    if (s_tail > s_head) s_tail = s_head;
    if (s_head - s_tail > FLASHLOG_CAPACITY - FLASHLOG_REC_PER_SECTOR)
        s_tail = s_head - (FLASHLOG_CAPACITY - FLASHLOG_REC_PER_SECTOR);

    ESP_LOGI(TAG, "recovered head=%lu tail=%lu (%lu pending)",
             (unsigned long)s_head, (unsigned long)s_tail,
             (unsigned long)(s_head - s_tail));
}

static void save_state(void)
{
    uint32_t i = 0;
    for (; i < STATE_ENTRIES; i++) {
        StateEntry e;
        extflash_read(i * sizeof(StateEntry), (uint8_t *)&e, sizeof(e));
        if (e.magic == 0xFFFFFFFFu) break;
    }
    if (i >= STATE_ENTRIES) { extflash_erase_sector(0); i = 0; }

    StateEntry e = {
        .magic = STATE_MAGIC,
        .head  = s_head,
        .tail  = s_tail,
        .pad   = 0xFFFF,
    };
    e.crc = rec_crc16((const uint8_t *)&e, 12);
    extflash_write_page(i * sizeof(StateEntry), (const uint8_t *)&e, sizeof(e));
}

bool flashlog_begin(bool cold_boot)
{
    s_ok = extflash_begin();
    if (!s_ok) return false;

    if (!cold_boot && s_valid == RTC_VALID_MAGIC) return true;

    recover();
    s_valid = RTC_VALID_MAGIC;
    return true;
}

bool flashlog_ok(void) { return s_ok; }

bool flashlog_append(LogRecord *r)
{
    if (!s_ok) return false;

    uint32_t addr = slot_addr(s_head);
    if (addr % EXTFLASH_SECTOR_SIZE == 0) {
        /* Entering a fresh sector: erase it, dropping the oldest records if the
         * ring is full (the tail may live in exactly this sector). */
        if (s_head - s_tail > FLASHLOG_CAPACITY - FLASHLOG_REC_PER_SECTOR)
            s_tail = s_head - (FLASHLOG_CAPACITY - FLASHLOG_REC_PER_SECTOR);
        extflash_erase_sector(addr);
    }

    r->seq = s_head;
    rec_seal(r);
    extflash_write_page(addr, (const uint8_t *)r, sizeof(*r));
    s_head++;
    return true;
}

uint32_t flashlog_head_seq(void) { return s_head; }
uint32_t flashlog_pending(void)  { return s_head - s_tail; }

bool flashlog_peek(uint32_t i, LogRecord *r)
{
    if (!s_ok || i >= flashlog_pending()) return false;
    extflash_read(slot_addr(s_tail + i), (uint8_t *)r, sizeof(*r));
    return rec_valid(r) && r->seq == s_tail + i;
}

void flashlog_advance(uint32_t n)
{
    s_tail += n;
    if (s_tail > s_head) s_tail = s_head;
    save_state();
}

void flashlog_sleep(void)
{
    if (s_ok) extflash_power_down();
}
