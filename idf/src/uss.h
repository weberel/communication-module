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
} uss_result_t;

/* Trigger one measurement and read the result block (CRC-verified).
 * Blocks up to USS_MEAS_TIMEOUT_MS while the slave measures; returns false on
 * absent module, timeout, or CRC failure (out is zeroed then). Note: a true
 * return with code != 122 means "module fine, no usable echo" -- the signal
 * metrics (amp/snr/gain) are still valid diagnostics. */
bool uss_sample(uss_result_t *out);

#ifdef __cplusplus
}
#endif
