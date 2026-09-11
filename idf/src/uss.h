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
    int32_t  flow_ulpm;   /* calibrated flow, uL/min */
    int32_t  dtof_ps;     /* delta time-of-flight, ps */
    int16_t  temp_cC;     /* gas temperature, 0.01 C */
    uint16_t amp_ups;     /* peak echo amplitude, upstream (ADC counts) */
    uint16_t amp_dns;     /* downstream */
    uint8_t  snr_db2;     /* SNR, dB x2 */
    uint8_t  gain;        /* PGA gain index used */
    uint32_t vol_ml;      /* totalized volume, mL (0 until autonomous mode) */
    /* Free-running capture counters (USS_REG_CAP_*). They WRAP; take the
     * difference between consecutive reads. All three zero = a module that
     * predates them. These are the only way to see the true failure rate: we
     * sample one capture per 5 min while the module runs ~300. */
    uint8_t  recoveries;         /* abs-ToF latch recoveries, wraps */
    uint16_t xt_applied_x10us;   /* settle the module applied, 10 us units */
    uint16_t cap_n;
    uint16_t cap_badcode;
    uint16_t cap_badsnr;
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

/* Stop autonomous mode; the module drops its 5 V boost and AFE rails with it.
 * Used by the power audit as a clean, bus-preserving load step. */
bool uss_stop_auto(void);

#ifdef __cplusplus
}
#endif
