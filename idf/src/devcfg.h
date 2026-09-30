/*
 * devcfg.h -- remote device configuration (downlink stage 1, com-0.35).
 *
 * The server publishes a RETAINED JSON payload on ecotrace/<user>/config; the
 * board reads it at the start of every upload session and applies it here.
 * Contract: ecotrace-server/docs/DOWNLINK.md. Plan: idf/PLAN_DOWNLINK.md.
 *
 * What is configurable in stage 1: the sample interval and the upload period.
 * Values live in NVS (namespace "cfg") and are cached in RTC RAM, so a wake
 * that does not upload never opens NVS. Compiled defaults from config.h apply
 * until the first payload arrives.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Result of feeding one payload to devcfg_apply(). */
typedef enum {
    DEVCFG_APPLIED = 0,   /* new cfg_ver, values stored and in effect */
    DEVCFG_SAME,          /* cfg_ver equals the stored one: nothing to do */
    DEVCFG_EMPTY,         /* zero-byte payload: "no config", keep NVS */
    DEVCFG_PARSE,         /* not JSON, or cfg_ver missing / not an integer */
    DEVCFG_RANGE,         /* an interval outside its bounds: whole payload dropped */
    DEVCFG_NVS,           /* parsed and in range, but the NVS write failed */
} devcfg_rc_t;

/* Initialise NVS and, on a cold boot (RTC cache invalid), load the stored
 * values. Call once, early in app_main, before anything asks for an interval. */
void devcfg_init(void);

/* Values in effect. Never zero. */
uint32_t devcfg_sample_s(void);
uint32_t devcfg_upload_s(void);

/* Version in effect; 0 if never configured. */
uint32_t devcfg_ver(void);

/* "parse" or "range" while the LAST RECEIVED payload was rejected, else NULL.
 * Cleared by the next payload that parses and is in range (applied or same). */
const char *devcfg_err(void);

/* Parse, bounds-check and store one payload. len may be 0 (empty retained
 * message). Safe to call from the MQTT event task: NVS is thread-safe and the
 * cache is written last, one word at a time. */
devcfg_rc_t devcfg_apply(const char *json, size_t len);

const char *devcfg_rc_name(devcfg_rc_t rc);

#ifdef __cplusplus
}
#endif
