/*
 * flash_log.h  --  persistent ring log on the GD25Q128, ESP-IDF port.
 * Semantics identical to the hardware-validated Arduino module
 * (src/datalogger/flash_log.*): sector 0 = cursor journal, sectors 1..4095 =
 * ring of self-validating LogRecords; head/tail survive deep sleep in RTC RAM
 * and are recovered from the chip after power loss. Since v4 records are
 * 128 B (was 64 B) -- old-geometry slots are simply invalid records to the
 * scanner, so a migrated board starts a fresh ring (Arduino build is frozen).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASHLOG_REC_SIZE       128u
#define FLASHLOG_DATA_BASE      4096u                 /* sector 1 */
#define FLASHLOG_DATA_SECTORS   4095u
#define FLASHLOG_REC_PER_SECTOR 32u                   /* 4096 / 128 */
#define FLASHLOG_CAPACITY       (FLASHLOG_DATA_SECTORS * FLASHLOG_REC_PER_SECTOR)

static_assert(sizeof(LogRecord) == FLASHLOG_REC_SIZE,
              "ring geometry must match the record size");

/* Wake the flash; on cold boot (RTC RAM lost) recover cursors from the chip. */
bool     flashlog_begin(bool cold_boot);
bool     flashlog_ok(void);

bool     flashlog_append(LogRecord *r);               /* fills seq/magic/crc */
uint32_t flashlog_head_seq(void);
uint32_t flashlog_pending(void);
bool     flashlog_peek(uint32_t i, LogRecord *r);     /* i-th pending, 0 = oldest */
void     flashlog_advance(uint32_t n);                /* persists the cursor */
void     flashlog_sleep(void);                        /* deep power-down */

#ifdef __cplusplus
}
#endif
