/*
 * devcfg.h -- remote device configuration (downlink, com-0.35 / com-0.36).
 *
 * The server publishes a RETAINED JSON payload on ecotrace/<user>/config; the
 * board reads it at the start of every upload session and applies it here.
 * Contract: ecotrace-server/docs/DOWNLINK.md. Plan: idf/PLAN_DOWNLINK.md.
 *
 * Stage 1 (0.35): sample interval and upload period, in NVS (namespace "cfg"),
 * cached in RTC RAM so a wake that does not upload never opens NVS. Compiled
 * defaults from config.h apply until the first payload arrives.
 * Stage 2 (0.36): the OTA target (fw_ver / fw_url / fw_sha256) from the same
 * payload, held for the session only, plus the per-version attempt counter
 * and the last OTA error, both in NVS because they must outlive a rollback.
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

/* Forget the RTC cache. Called right before the restart into a new image,
 * whose RTC layout may differ; it reloads from NVS on its cold boot. */
void devcfg_drop_cache(void);

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

/* ---- OTA target and bookkeeping (stage 2) -------------------------------- */

typedef struct {
    char ver[32];
    char url[200];
    char sha256[65];
} devcfg_fw_t;

/* The fw_* triple from the last accepted payload of THIS boot, or NULL when
 * the payload had none (or an incomplete set, which counts as none). */
const devcfg_fw_t *devcfg_fw_target(void);

/* Download attempts recorded for this version (NVS). */
uint8_t devcfg_fw_tries(const char *ver);

/* Record one more attempt for ver. A different version than the one on
 * record starts at 1 and clears a stale fw_err. */
void devcfg_fw_note_try(const char *ver);

/* Last OTA failure for the status message: word from the contract
 * (download | sha256 | write | rollback) and the version it was for.
 * NULL when there is none. */
const char *devcfg_fw_err(void);
const char *devcfg_fw_err_ver(void);
void devcfg_fw_set_err(const char *err, const char *ver);
void devcfg_fw_clear_err(void);

/* A rolled-back image was found in the other slot. Records fw_err "rollback"
 * for ver; returns true only the first time this particular rollback is seen. */
bool devcfg_fw_note_rollback(const char *ver);

#ifdef __cplusplus
}
#endif
