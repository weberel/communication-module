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

    /* ---- session phase timing, ms since the modem rail came up ------------
     * CUMULATIVE, not per-phase: each is the stopwatch reading when that phase
     * completed, so a phase's own cost is the difference from the one before
     * and a phase that never ran reads 0. Publishing cumulative values means a
     * truncated session still says how far it got.
     *
     * t_pub_max_ms is the decisive one when the drain is slow: a large total
     * with a small max is the broker throttling every publish evenly, while a
     * large max is ONE stall -- those have completely different fixes, and the
     * average hides both. */
    uint32_t t_modem_ms;       /* rail on -> modem answers AT                 */
    uint32_t t_reg_ms;         /* -> EPS registered (+CEREG 1/5)              */
    uint32_t t_ppp_ms;         /* -> PPP has an IP                            */
    uint32_t t_sntp_ms;        /* -> SNTP done (or gave up)                   */
    uint32_t t_mqtt_ms;        /* -> broker connected                         */
    uint32_t t_drain_ms;       /* -> backlog drained (or the drain broke)     */
    uint32_t t_total_ms;       /* -> rail off                                 */
    uint32_t t_pub_max_ms;     /* slowest single publish in the drain         */
    uint16_t n_batches;        /* publishes issued during the drain           */
    bool     drain_broke;      /* a publish went unacked and stopped the drain */

    /* ---- supply UNDER LOAD (power doc B2) --------------------------------
     * Sampled mid-drain, with the modem registered and transmitting. Every
     * other VBAT/VSYS reading on this board is taken with the modem off, so
     * the sag under the A7672's ~2 A burst has never been measured -- and that
     * sag, not energy, is what sets the minimum battery size.
     *
     * VBAT is the cell terminal, VSYS is what the ESP32 and the ultrasonic
     * module actually run on. Reporting BOTH is the point: the difference
     * between them under load is series resistance between cell and board,
     * which is precisely the failure we cannot otherwise distinguish from a
     * tired cell. */
    uint16_t vbat_load_mv;     /* VBAT during the drain, 0 if never sampled   */
    uint16_t vsys_load_mv;     /* VSYS during the drain                       */
    uint16_t vbat_pre_mv;      /* VBAT just before the modem rail came up     */
    uint8_t  fault0, fault1;   /* BQ25792 REG20/21 latched during the session */
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
    int8_t   uss_preset;       /* gas preset on the module: -1 not tried this
                                * module boot, 0 failed, 1 in place */
} uplink_ctx_t;

uplink_result_t uplink_upload_all(const uplink_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
