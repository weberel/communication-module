/*
 * record.h  --  the on-flash log record.
 *
 * One fixed 64-byte record per sample. Fixed size keeps the flash ring trivial
 * (64 records per 4 KB sector, 4 per 256 B page, never crosses a page) and makes
 * capacity predictable: 4095 data sectors x 64 = 262,080 records = ~2.5 years at
 * one sample per 5 minutes.
 *
 * Every record is self-validating (magic + seq + CRC) so the log can be recovered
 * by scanning after a power loss, with no filesystem underneath.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>   /* static_assert in C11+ */

#define REC_MAGIC 0x45434C32u   /* "ECL2" -- bumped when the layout changed
                                 * (v2: +MS5837 pressure/temperature); records
                                 * with the old magic are simply not recognised */

/* flags bits */
#define RECF_SOLAR    0x01   /* solar input present (VAC2) */
#define RECF_USB      0x02   /* USB input present (VAC1) */
#define RECF_WEATHER  0x04   /* weather classified good this day */
#define RECF_ECO_CHG  0x08   /* charge target limited to ~80 % SoC */

typedef struct __attribute__((packed)) {
    uint32_t magic;        /* REC_MAGIC */
    uint32_t seq;          /* monotonic record number (ring position = seq % capacity) */
    uint32_t ts_s;         /* unix seconds at sample time; 0 = clock not yet synced */
    uint32_t uptime_s;     /* seconds since cold boot (diagnostic) */
    uint8_t  boot_id;      /* random per cold boot (diagnostic) */
    uint8_t  flags;        /* RECF_* */
    uint8_t  chg_stat;     /* BQ25792::ChgStat */
    uint8_t  soc_pct;      /* crude voltage-based SoC */
    uint16_t vbat_mv;
    int16_t  ibat_ma;      /* + charging, - discharging */
    uint16_t vbus_mv;
    int16_t  ibus_ma;      /* + into the charger */
    uint16_t vac2_mv;      /* solar input voltage */
    uint16_t vsys_mv;
    uint16_t vindpm_mv;    /* MPPT operating point */
    uint16_t vreg_mv;      /* active charge-voltage target (4200 or eco) */
    uint8_t  fault0;       /* BQ25792 REG20 */
    uint8_t  fault1;       /* BQ25792 REG21 */
    uint16_t harvest_mah;  /* solar charge harvested so far today */
    uint16_t light_ch0;    /* LTR-303 visible+IR */
    uint16_t light_ch1;    /* LTR-303 IR */
    int16_t  acc_mg[3];    /* SC7A20 X/Y/Z */
    uint16_t press_dmbar;  /* MS5837 pressure, 0.1 mbar units (0 = no sensor) */
    int16_t  temp_cC;      /* MS5837 temperature, 0.01 C units */
    uint8_t  rsvd[8];      /* spare for future fields (0xFF) */
    uint16_t crc;          /* CRC16-CCITT over bytes [0 .. offsetof(crc)-1] */
} LogRecord;

static_assert(sizeof(LogRecord) == 64, "LogRecord must be exactly 64 bytes");

/* CRC16-CCITT (0x1021, init 0xFFFF) -- small and good enough for 62 bytes. */
static inline uint16_t rec_crc16(const uint8_t* p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void rec_seal(LogRecord* r)
{
    r->magic = REC_MAGIC;
    r->crc   = rec_crc16((const uint8_t*)r, offsetof(LogRecord, crc));
}

static inline bool rec_valid(const LogRecord* r)
{
    return r->magic == REC_MAGIC &&
           r->crc == rec_crc16((const uint8_t*)r, offsetof(LogRecord, crc));
}
