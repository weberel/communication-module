/*
 * uss.h  --  master-side driver for the ultrasonic flow module
 * (MSP430FR6043 eUSCI_B0 I2C slave, contract in uss_link.h).
 *
 * Optional like every other sensor: on boards without the gas cell the first
 * transaction NACKs (~1 ms) and the sample degrades to zeros, never errors.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  status;      /* raw STATUS register (BOOT bit = slave rebooted) */
    uint8_t  code;        /* USS message code, 122 = valid measurement */
    uint16_t seq;         /* slave-side measurement counter */
    uint8_t  recoveries;  /* abs-ToF re-searches the module has forced.
                           * A latch is INVISIBLE from here otherwise: a
                           * wrongly-locked module answers every read
                           * correctly while emitting code 135, so it looks
                           * healthy. This is the only way to tell "recovery
                           * fired and worked" from "never slipped". */
    int32_t  flow_ulpm;   /* calibrated flow, uL/min */
    int32_t  dtof_ps;     /* delta time-of-flight, ps */
    int16_t  temp_cC;     /* gas temperature, 0.01 C */
    uint16_t amp_ups;     /* peak echo amplitude, upstream (ADC counts) */
    uint16_t amp_dns;     /* downstream */
    uint8_t  snr_db2;     /* SNR, dB x2 */
    uint8_t  gain;        /* PGA gain index used */
    uint32_t vol_ml;      /* totalized volume, mL (0 until autonomous mode) */
    /* Absolute time-of-flight, RAW Q40 seconds exactly as the USS library
     * produced it -- deliberately not scaled here. This is the speed-of-sound
     * observable (gas composition), so the conversion is applied off-device:
     *   microseconds = raw * 1e6 / 2^40   (= raw / 1099511.627776)
     * Keeping it raw also stops a wrong exponent being baked into two
     * firmwares, which is a documented trap on this project. */
    uint32_t tof_ups_q40;
    uint32_t tof_dns_q40;
} uss_result_t;

/* Trigger one measurement and read the result block (CRC-verified).
 * Blocks up to USS_MEAS_TIMEOUT_MS while the slave measures; returns false on
 * absent module, timeout, or CRC failure (out is zeroed then). Note: a true
 * return with code != 122 means "module fine, no usable echo" -- the signal
 * metrics (amp/snr/gain) are still valid diagnostics. */
bool uss_sample(uss_result_t *out);

/* Start autonomous measurement + totalizing on the module (period in seconds).
 * Idempotent and cheap; call it each wake so the mode is restored automatically
 * after the module reboots. With this running, uss_sample() stops commanding
 * measurements and simply reads the latest latched block plus vol_ml. */
bool uss_start_auto(uint16_t period_s);

/* Link health, read from the module registers 0x30..0x3F (own CRC).
 *
 * `starts` is the diagnostic that matters after a failed read: it counts I2C
 * address matches the module saw. If it advanced by roughly the number of
 * transactions we attempted, the bus reached the module and the fault is above
 * the physical layer. If it did not advance at all, we never got on the bus.
 * `rst_cause` is the raw MSP430 SYSRSTIV latched at boot: 0x0002 brownout,
 * 0x000E SVSH, 0x0014 PMMSWPOR. */
typedef struct {
    uint16_t rst_cause;
    uint16_t starts, stops;
    uint16_t rx_bytes, tx_bytes;
    uint16_t cmds;
    uint16_t uptime_s;
    uint8_t  last_cmd;
} uss_health_t;

bool uss_read_health(uss_health_t *out);

/* Raw totalizer, module registers 0x50..0x67 (PROTO 3, own CRC).
 *
 * S1 and S0 are ABSOLUTE running sums in the module's FRAM, integrated at the
 * capture rate and carrying NO calibration constants -- see the derivation in
 * uss_link.h. Take deltas between reads and apply the constants here (or on the
 * server), where OTA can reach them:
 *
 *     V_actual = K * ( dS1/2^24  -  offset_ps * dS0/2^40 )
 *     V_std    = V_actual * (P_mbar/1013.25) * (293.15/T_kelvin)
 *
 * Because S0 is the exact sensitivity the offset would have had at 1 Hz, the
 * zero can be changed AFTER the fact and past windows still recompute
 * correctly. Prefer that over burning an offset into module FRAM with
 * uss_zerocal().
 *
 * `skip` counts captures in the window that produced no usable ToF. It is what
 * separates a genuinely low-flow window from one where the meter was blind --
 * without it the two look identical. A window with skip comparable to n is not
 * a low reading, it is a missing one. */
typedef struct {
    int64_t  s1;        /* sum dtof_ps*dt_ms/(tu_ns*td_ns), Q24 */
    int64_t  s0;        /* sum      dt_ms/(tu_ns*td_ns), Q40    */
    uint32_t n;         /* valid captures integrated            */
    uint16_t skip;      /* captures with no usable ToF          */
    uint8_t  flags;     /* USS_TOT_FLAG_SAT = an add saturated  */
} uss_totals_t;

/* Read the raw totalizer block (CRC-verified). Returns false on an absent
 * module, a CRC failure, or a module older than PROTO 3 -- in which case the
 * VOL_ML path in uss_result_t is still valid and should be used instead. */
bool uss_read_totalizer(uss_totals_t *out);

/* PROTO version the module reported on the last successful uss_sample(), or 0
 * if it has never been reached. 3 and above carry the raw totalizer. */
uint8_t uss_proto(void);


/* Zero the module totalizer. It persists in the module FRAM now, so nothing
 * clears it implicitly any more -- not even a recovery power-cycle. */
bool uss_reset_volume(void);

/* Reset the slave state machine without dropping its rail -- the rung above a
 * power cycle, and the one that keeps the persisted totalizer. */
bool uss_soft_reset(void);

/* Null the module dTOF offset at the CURRENT flow and persist it there.
 * Only valid at genuine zero flow; the module cannot check that. */
bool uss_zerocal(void);

#ifdef __cplusplus
}
#endif
