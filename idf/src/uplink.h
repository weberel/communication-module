/*
 * uplink.h  --  store-and-forward to ThingsBoard over MQTTS. TLS ONLY.
 *
 * Architecture (the whole point of the IDF port): both transports are plain
 * netifs -- cellular via esp_modem PPP, WiFi via esp_wifi -- and ONE mbedTLS +
 * esp-mqtt stack runs on top. There is no plaintext code path to fall back to:
 * if TLS cannot be established, records stay banked in the flash ring, which
 * costs nothing (proven by the basement weekend on the Arduino build).
 *
 * Policy (2026-08-11 spec): cellular primary, WiFi backup. Escalation changes
 * TECHNIQUE not TEMPO -- fail fast when there is no signal, never retry more
 * often than the schedule, hardware-kick the modem between failed attempts.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flash_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     any_success;
    bool     all_sent;
    bool     used_wifi;
    uint32_t sent;
    int      cell_rssi_dbm;    /* 0 if modem never attached */
    int      wifi_rssi_dbm;    /* 0 if WiFi unused */
    uint8_t  cell_reg_stat;    /* last +CEREG stat seen (registration diagnostics) */
} uplink_result_t;

typedef struct {
    uint8_t  boot_id;
    uint8_t  reset_reason;
    uint16_t boot_count;
    uint16_t wake_count;
    uint16_t crash_count;
    uint16_t vbat_mv;          /* gates: cellular >= MODEM_MIN, wifi >= WIFI_MIN */
    bool     deep_search;      /* once/day: use the long attach timeout */
    bool     skip_cellular;    /* previous attempt crashed the board */
    uint8_t  sun_hours;        /* weather diagnostics for the status record */
    uint16_t voc_max_mv;
    uint32_t uptime_s;         /* uptime at the newest record: the reference for
                                * back-dating records logged before a clock sync */
    /* Module link health, read over I2C from regs 0x30..0x3F.
     *
     * Carried in the STATUS record, NOT in LogRecord: LogRecord is CRC'd and
     * stored in external flash, so widening it would invalidate every record
     * already buffered there. This is session diagnostics, not per-sample
     * measurement, so the status record is where it belongs anyway. */
    bool     uss_health_valid;
    uint16_t uss_rst_cause;    /* raw SYSRSTIV: 2 brownout, 0xE SVSH, 0x14 SWPOR */
    uint16_t uss_lh_starts;    /* I2C address matches the module has seen */
    uint16_t uss_lh_uptime_s;  /* module uptime; a drop means it restarted */
} uplink_ctx_t;

uplink_result_t uplink_upload_all(const uplink_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
