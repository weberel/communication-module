/*
 * flash_log.h  --  persistent ring log on the GD25Q128, ESP-IDF port.
 * Semantics identical to the hardware-validated Arduino module
 * (src/datalogger/flash_log.*): sector 0 = cursor journal, sectors 1..4095 =
 * ring of 64 B self-validating LogRecords; head/tail survive deep sleep in
 * RTC RAM and are recovered from the chip after power loss. Storage format is
 * bit-compatible -- a board migrated from the Arduino build keeps its data.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "record.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASHLOG_REC_SIZE       64u
#define FLASHLOG_DATA_BASE      4096u                 /* sector 1 */
#define FLASHLOG_DATA_SECTORS   4095u
#define FLASHLOG_REC_PER_SECTOR 64u                   /* 4096 / 64 */
#define FLASHLOG_CAPACITY       (FLASHLOG_DATA_SECTORS * FLASHLOG_REC_PER_SECTOR)

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
