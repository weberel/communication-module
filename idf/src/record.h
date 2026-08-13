/*
 * record.h  --  the on-flash log record.
 *
 * One fixed 128-byte record per sample. Fixed size keeps the flash ring trivial
 * (32 records per 4 KB sector, 2 per 256 B page, never crosses a page) and makes
 * capacity predictable: 4095 data sectors x 32 = 131,040 records = ~1.25 years
 * at one sample per 5 minutes.
 *
 * Every record is self-validating (magic + seq + CRC) so the log can be recovered
 * by scanning after a power loss, with no filesystem underneath.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>   /* static_assert in C11+ */

#define REC_MAGIC 0x45434C34u   /* "ECL4" -- bumped when the layout changed
                                 * (v4: 64 -> 128 B, +ultrasonic flow module
                                 * and WF280A raw pressure, 2026-08-13; v3
                                 * added the charger thermal diagnostics).
                                 * Records with old magics are not recognised,
                                 * so flash this build right after a clean
                                 * upload (pending backlog would be dropped). */

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
    int16_t  tdie_dC;      /* BQ25792 die temperature, 0.1 C units */
    uint16_t ts_pct_x100;  /* battery NTC (TS pin) as % of bias, x100 */
    uint8_t  ts_stat;      /* BQ Charger_Status_4: JEITA verdict bits */
    uint8_t  sensor_ok;    /* bit0 BQ, bit1 LTR303, bit2 SC7A20, bit3 MS5837,
                            * bit4 USS flow module, bit5 WF280A -- set only when
                            * the source answered correctly, so a zero reading
                            * is distinguishable from a dead sensor */
    /* --- v4: ultrasonic flow module (MSP430FR6043 I2C slave, uss_link.h)
     *         + WF280A raw pressure. All zero when absent (see sensor_ok). --- */
    int32_t  uss_flow_ulpm;  /* calibrated flow, uL/min */
    int32_t  uss_dtof_ps;    /* delta time-of-flight, ps (the raw flow signal) */
    int16_t  uss_temp_cC;    /* gas temperature, 0.01 C */
    uint16_t uss_amp_ups;    /* peak echo amplitude, upstream (ADC counts) */
    uint16_t uss_amp_dns;    /* downstream */
    uint8_t  uss_code;       /* USS message code; 122 = valid, 126 = no echo */
    uint8_t  uss_gain;       /* PGA gain index used */
    uint8_t  uss_snr_db2;    /* SNR, dB x2 */
    uint8_t  uss_status;     /* raw STATUS register (BOOT bit = slave rebooted) */
    uint32_t uss_vol_ml;     /* totalized volume, mL (0 until autonomous mode) */
    uint32_t wf_praw;        /* WF280A raw 24-bit pressure counts */
    uint32_t wf_traw;        /* WF280A raw 24-bit temperature counts */
    uint8_t  rsvd[36];     /* spare for future fields (0xFF) */
    uint16_t crc;          /* CRC16-CCITT over bytes [0 .. offsetof(crc)-1] */
} LogRecord;

static_assert(sizeof(LogRecord) == 128, "LogRecord must be exactly 128 bytes");

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
