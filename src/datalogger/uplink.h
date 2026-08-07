/*
 * uplink.h  --  store-and-forward upload to ThingsBoard.
 *
 * Drains the flash log to POST_URL (a ThingsBoard device-telemetry endpoint,
 * http://<host>/api/v1/<token>/telemetry) in batches of timestamped records:
 *     [ {"ts":<unix ms>,"values":{...}}, ... ]
 *
 * Transport order: SIMCom A7672E cellular first; if the modem fails at any stage
 * (boot, SIM, registration, PDP, POST) the remaining records go over WiFi
 * (WIFI_SSID/WIFI_PASS in secrets.h) as backup. Both transports also sync the
 * system clock (modem NITZ / SNTP), which timestamps all future records.
 *
 * The flash cursor only advances after a batch POSTs successfully, so records are
 * never lost to a failed upload -- they simply go out next time.
 */
#pragma once

#include <Arduino.h>
#include "flash_log.h"

namespace Uplink {

struct Result {
    bool     any_success;    /* at least one batch delivered */
    bool     all_sent;       /* backlog fully drained */
    bool     used_wifi;
    uint32_t sent;
    int      rssi_dbm;       /* cellular RSSI if the modem came up, else 0 */
};

/* Device-health telemetry sent as one extra record with every upload. This is
 * the only debugger a deployed unit has -- when something misbehaves in the
 * field, these numbers are what you diagnose from. */
struct StatusInfo {
    uint8_t  boot_id;
    uint8_t  reset_reason;   /* esp_reset_reason() of this boot */
    uint16_t boot_count;     /* boots since cold start (any cause) */
    uint16_t wake_count;     /* clean timer wakes since cold start */
    uint16_t crash_count;    /* consecutive abnormal resets (0 = healthy) */
    uint16_t wdt_trips;      /* watchdog/panic reboots since cold start */
    uint16_t upload_fails;   /* consecutive failed upload attempts */
    uint16_t vbat_min_mv;    /* battery envelope since the last upload */
    uint16_t vbat_max_mv;
    uint32_t awake_ms;       /* time spent awake since the last upload */
    uint32_t uptime_s;
};

Result uploadAll(FlashLog& log, uint32_t interval_s, const StatusInfo& info);

}  // namespace Uplink
