#include "uplink.h"
#include "config.h"
#include "timeutil.h"
#include "ModemA7672.h"
#include "esp_task_wdt.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#if defined(WIFI_SSID) && defined(WIFI_PASS)
#define UPLINK_HAS_WIFI 1
#include <WiFi.h>
#include <HTTPClient.h>
#else
#define UPLINK_HAS_WIFI 0
#endif

namespace Uplink {

#ifdef POST_URL

static char s_json[4608];   /* one batch of UPLOAD_BATCH_RECORDS records */

/* ---- JSON building ------------------------------------------------------- */

/* Timestamp for a record: exact if the clock was synced when it was sampled,
 * otherwise back-computed from its distance to the newest record (sampled just
 * before this upload) assuming the regular interval. */
static int64_t recordTs(const LogRecord& r, int64_t now_ms, uint32_t head_seq,
                        uint32_t interval_s)
{
    if (r.ts_s) return (int64_t)r.ts_s * 1000;
    return now_ms - (int64_t)(head_seq - 1 - r.seq) * interval_s * 1000;
}

static int recordValues(const LogRecord& r, char* out, size_t cap)
{
    int32_t bat_mw = (int32_t)r.vbat_mv * r.ibat_ma / 1000;
    return snprintf(out, cap,
        "\"vbat_mv\":%u,\"ibat_ma\":%d,\"bat_mw\":%ld,\"soc_pct\":%u,"
        "\"vbus_mv\":%u,\"ibus_ma\":%d,\"vac2_mv\":%u,\"vsys_mv\":%u,"
        "\"vindpm_mv\":%u,\"vreg_mv\":%u,\"chg_stat\":%u,"
        "\"fault0\":%u,\"fault1\":%u,\"harvest_mah\":%u,"
        "\"solar\":%u,\"usb\":%u,\"weather_good\":%u,\"eco_chg\":%u,"
        "\"light_ch0\":%u,\"light_ch1\":%u,"
        "\"acc_x_mg\":%d,\"acc_y_mg\":%d,\"acc_z_mg\":%d,"
        "\"press_mbar\":%.1f,\"temp_c\":%.2f",
        r.vbat_mv, r.ibat_ma, (long)bat_mw, r.soc_pct,
        r.vbus_mv, r.ibus_ma, r.vac2_mv, r.vsys_mv,
        r.vindpm_mv, r.vreg_mv, r.chg_stat,
        r.fault0, r.fault1, r.harvest_mah,
        (r.flags & RECF_SOLAR) ? 1 : 0, (r.flags & RECF_USB) ? 1 : 0,
        (r.flags & RECF_WEATHER) ? 1 : 0, (r.flags & RECF_ECO_CHG) ? 1 : 0,
        r.light_ch0, r.light_ch1,
        r.acc_mg[0], r.acc_mg[1], r.acc_mg[2],
        r.press_dmbar / 10.0f, r.temp_cC / 100.0f);
}

/* Batch of n pending records -> ThingsBoard array. Returns records included
 * (may be < n if one fails to read; corrupt records are skipped, not fatal). */
static uint32_t buildBatch(FlashLog& log, uint32_t n, int64_t now_ms,
                           uint32_t interval_s)
{
    size_t len = 0;
    uint32_t included = 0;
    s_json[len++] = '[';
    for (uint32_t i = 0; i < n; i++) {
        LogRecord r;
        if (!log.peek(i, r)) continue;
        char vals[512];
        recordValues(r, vals, sizeof(vals));
        int w;
        if (now_ms > 0) {
            w = snprintf(s_json + len, sizeof(s_json) - len - 2,
                         "%s{\"ts\":%lld,\"values\":{%s}}",
                         included ? "," : "",
                         (long long)recordTs(r, now_ms, log.headSeq(), interval_s),
                         vals);
        } else {
            /* No clock from any source: send bare values, server stamps them. */
            w = snprintf(s_json + len, sizeof(s_json) - len - 2,
                         "%s{%s}", included ? "," : "", vals);
        }
        if (w < 0 || (size_t)w >= sizeof(s_json) - len - 2) break;
        len += (size_t)w;
        included++;
    }
    s_json[len++] = ']';
    s_json[len] = '\0';
    return included;
}

/* ---- transports ---------------------------------------------------------- */

static ModemA7672 s_modem;

static bool cellPost(const char* json)
{
    int status = 0;
    char body[200] = "";
    bool ok = s_modem.httpPost(POST_URL, "application/json", json, status,
                               body, sizeof(body));
    if (!ok)
        /* status <100 = AT-flow failure before any HTTP happened; 7xx = SIMCom
         * internal (703 DNS, 706 timeout, ...); else a real HTTP error code and
         * the body is the server's explanation. */
        Serial.printf("  cell POST failed (status %d) body: %.160s\n", status, body);
    return ok;
}

static bool cellUp(int& rssi_dbm)
{
    Serial.println("uplink: cellular...");
    esp_task_wdt_reset();
    if (!s_modem.begin()) { Serial.println("  modem did not boot"); return false; }
    esp_task_wdt_reset();
    if (!s_modem.simReady() || !s_modem.waitForNetwork(45000)) {
        Serial.println("  no SIM / no network"); return false;
    }
    esp_task_wdt_reset();
    if (!s_modem.connectGPRS(SIM_APN)) { Serial.println("  PDP failed"); return false; }
    rssi_dbm = s_modem.signalQuality_dBm();
    Serial.printf("  attached, %d dBm\n", rssi_dbm);

    /* Re-sync the clock on every upload -- the C6's sleep timer runs off an RC
     * oscillator, so between uploads the wall clock drifts by minutes. */
    int64_t t = s_modem.getUnixTimeMs();
    if (t > 0) setClockMs(t);
    return true;
}

#if UPLINK_HAS_WIFI
static bool wifiPost(const char* json)
{
    HTTPClient http;
    if (!http.begin(POST_URL)) return false;
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)json, strlen(json));
    http.end();
    bool ok = code >= 200 && code < 300;
    if (!ok) Serial.printf("  wifi POST failed (%d, RSSI %d dBm)\n", code, WiFi.RSSI());
    return ok;
}

static int s_wifi_rssi;   /* captured on connect for the status record */

static bool wifiUp()
{
    Serial.println("uplink: wifi backup...");
    esp_task_wdt_reset();
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   /* full throughput for the short upload burst */
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(250);
    if (WiFi.status() != WL_CONNECTED) { Serial.println("  no wifi"); return false; }
    s_wifi_rssi = WiFi.RSSI();
    Serial.printf("  connected, %d dBm\n", s_wifi_rssi);

    /* Always re-sync, not just on first boot -- see cellUp(). SNTP updates the
     * system clock in the background once it gets an answer. */
    configTime(0, 0, NTP_SERVER);
    if (!clockValid()) {
        t0 = millis();
        while (!clockValid() && millis() - t0 < 8000) delay(250);
    }
    return true;
}

static void wifiDown()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}
#endif

/* Drain the backlog through one transport; stops at the first failed POST. */
static uint32_t drain(FlashLog& log, uint32_t interval_s, bool (*post)(const char*))
{
    uint32_t sent = 0;
    while (log.pendingCount() > 0) {
        esp_task_wdt_reset();   /* one feed per batch; a stuck POST still trips */
        int64_t now_ms = nowMs();
        /* Without a clock the records can't carry timestamps -> singles. */
        uint32_t n = min(log.pendingCount(),
                         (uint32_t)(now_ms > 0 ? UPLOAD_BATCH_RECORDS : 1));
        uint32_t included = buildBatch(log, n, now_ms, interval_s);
        if (included == 0) { log.advance(n); continue; }   /* skip corrupt slots */
        if (!post(s_json)) {
            /* One retry per batch: on a marginal link (WiFi at -88 dBm drops
             * single POSTs) this rescues the drain instead of aborting it. */
            esp_task_wdt_reset();
            if (!post(s_json)) {
                Serial.printf("  drain stopped at seq %lu (%lu still pending)\n",
                              (unsigned long)(log.headSeq() - log.pendingCount()),
                              (unsigned long)log.pendingCount());
                break;
            }
        }
        log.advance(n);
        sent += included;
        /* Pace the server: draining a large backlog back-to-back trips what
         * looks like ThingsBoard Cloud rate limiting (intermittent 500s). */
        if (log.pendingCount() > 0) delay(UPLOAD_BATCH_GAP_MS);
    }
    return sent;
}

static const char* resetReasonName(uint8_t r)
{
    switch (r) {
        case 1:  return "poweron";     /* ESP_RST_POWERON */
        case 3:  return "sw";          /* ESP_RST_SW */
        case 4:  return "panic";       /* ESP_RST_PANIC */
        case 5:  return "int_wdt";     /* ESP_RST_INT_WDT */
        case 6:  return "task_wdt";    /* ESP_RST_TASK_WDT */
        case 7:  return "wdt";         /* ESP_RST_WDT */
        case 8:  return "deepsleep";   /* ESP_RST_DEEPSLEEP */
        case 9:  return "brownout";    /* ESP_RST_BROWNOUT */
        case 11: return "usb";         /* ESP_RST_USB */
        case 12: return "jtag";        /* ESP_RST_JTAG */
        default: return "other";
    }
}

/* One extra device-health record per upload session. Best-effort. */
static void sendStatus(bool (*post)(const char*), const Result& r, FlashLog& log,
                       const StatusInfo& si)
{
    const char* transport = (r.sent_cell && r.sent_wifi) ? "cell+wifi" :
                            r.sent_cell ? "cell" : r.sent_wifi ? "wifi" : "none";
    char values[560];
    snprintf(values, sizeof(values),
             "\"rssi_dbm\":%d,\"wifi_rssi_dbm\":%d,\"transport\":\"%s\","
             "\"sent_cell\":%lu,\"sent_wifi\":%lu,\"via_wifi\":%u,\"backlog\":%lu,"
             "\"boot_id\":%u,\"reset_reason\":\"%s\","
             "\"boot_count\":%u,\"wake_count\":%u,"
             "\"crash_count\":%u,\"wdt_trips\":%u,\"upload_fails\":%u,"
             "\"vbat_min_mv\":%u,\"vbat_max_mv\":%u,"
             "\"awake_ms\":%lu,\"uptime_s\":%lu,"
             "\"heap_min_free\":%lu,\"fw\":\"" FW_VERSION "\"",
             r.rssi_dbm, r.wifi_rssi_dbm, transport,
             (unsigned long)r.sent_cell, (unsigned long)r.sent_wifi,
             r.used_wifi ? 1 : 0, (unsigned long)log.pendingCount(),
             si.boot_id, resetReasonName(si.reset_reason),
             si.boot_count, si.wake_count,
             si.crash_count, si.wdt_trips, si.upload_fails,
             si.vbat_min_mv, si.vbat_max_mv,
             (unsigned long)si.awake_ms, (unsigned long)si.uptime_s,
             (unsigned long)esp_get_minimum_free_heap_size());

    int64_t now_ms = nowMs();
    if (now_ms > 0)
        snprintf(s_json, sizeof(s_json), "{\"ts\":%lld,\"values\":{%s}}",
                 (long long)now_ms, values);
    else
        snprintf(s_json, sizeof(s_json), "{%s}", values);
    post(s_json);
}

Result uploadAll(FlashLog& log, uint32_t interval_s, const StatusInfo& info,
                 bool skip_cellular)
{
    Result r = {};

    /* 1. Cellular (primary), unless it just crashed the board. */
    if (skip_cellular) {
        Serial.println("uplink: skipping cellular (last attempt crashed) -- wifi only");
    } else {
        bool cell_ok = cellUp(r.rssi_dbm);
        if (cell_ok) {
            uint32_t sent = drain(log, interval_s, cellPost);
            r.sent      += sent;
            r.sent_cell += sent;
            if (sent) r.any_success = true;
            if (log.pendingCount() == 0) sendStatus(cellPost, r, log, info);
        }
        s_modem.powerOff();
    }

    /* 2. WiFi backup for whatever cellular didn't deliver. */
#if UPLINK_HAS_WIFI
    if (log.pendingCount() > 0) {
        if (wifiUp()) {
            r.used_wifi     = true;
            r.wifi_rssi_dbm = s_wifi_rssi;
            uint32_t sent = drain(log, interval_s, wifiPost);
            r.sent      += sent;
            r.sent_wifi += sent;
            if (sent) r.any_success = true;
            if (log.pendingCount() == 0) sendStatus(wifiPost, r, log, info);
        }
        wifiDown();
    }
#endif

    r.all_sent = (log.pendingCount() == 0);
    Serial.printf("uplink: %lu records sent%s, %lu still pending\n",
                  (unsigned long)r.sent, r.used_wifi ? " (wifi used)" : "",
                  (unsigned long)log.pendingCount());
    return r;
}

#else  /* no POST_URL -> bench mode */

Result uploadAll(FlashLog& log, uint32_t, const StatusInfo&)
{
    Result r = {};
    Serial.printf("uplink: bench mode (no POST_URL in secrets.h) -- dropping %lu records\n",
                  (unsigned long)log.pendingCount());
    log.advance(log.pendingCount());
    r.any_success = r.all_sent = true;
    return r;
}

#endif

}  // namespace Uplink
