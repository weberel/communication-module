#include "uplink.h"
#include "config.h"
#include "record.h"

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_modem_api.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "board.h"
#include "bq25792.h"
#include "secrets.h"

static const char *TAG = "uplink";

/* ISRG Root X1 CA, compiled in from isrg_cert.c (generated from the PEM). */
extern const char isrg_root_pem[];
#define isrg_root_pem_start isrg_root_pem

/* ---- timeouts / policy ---- */
#define MODEM_UART_TX        20
#define MODEM_UART_RX        21
#define ATTACH_TIMEOUT_S     45      /* normal attach budget */
#define ATTACH_DEEP_S        180     /* once-a-day deep search */
#define NO_SIGNAL_FAILFAST_S 25      /* CSQ still 99 after this -> abort attempt */
#define MQTT_CONNECT_TO_MS   30000
#define PUBLISH_TO_MS        20000
/* Back to 8 after the telemetry trim: 22 datapoints per record x 8 = ~176 per
 * publish, comfortably under what made ThingsBoard Cloud drop the connection
 * (that was ~320, at 40 datapoints x 8). Larger batches drain a backlog in
 * fewer round trips, which matters over a slow cellular link. */
#define BATCH_RECORDS        4       /* starting batch; drain() shrinks on failure */
#define BATCH_GAP_MS         400     /* ThingsBoard Cloud dislikes bursts */

#define CLOCK_MIN 1767225600L        /* 2026-01-01 */
#define CLOCK_MAX 2082758400L        /* 2036-01-01 */

static bool clock_valid(void)
{
    time_t t = time(NULL);
    return t >= CLOCK_MIN && t < CLOCK_MAX;
}

/* ===================== MQTT publisher (transport-agnostic) ===================== */

static EventGroupHandle_t s_ev;
#define EV_MQTT_UP   BIT0
#define EV_MQTT_FAIL BIT1
#define EV_PUBACK    BIT2
#define EV_IP_UP     BIT3

static esp_mqtt_client_handle_t s_mqtt;
static volatile int s_pending_msg_id = -1;

static void mqtt_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:    xEventGroupSetBits(s_ev, EV_MQTT_UP);   break;
    case MQTT_EVENT_ERROR:
    case MQTT_EVENT_DISCONNECTED: xEventGroupSetBits(s_ev, EV_MQTT_FAIL); break;
    case MQTT_EVENT_PUBLISHED:
        if (e->msg_id == s_pending_msg_id) xEventGroupSetBits(s_ev, EV_PUBACK);
        break;
    default: break;
    }
}

static bool mqtt_up(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = TB_MQTT_URI,
        .broker.verification.certificate = isrg_root_pem_start,
        .credentials.username = TB_ACCESS_TOKEN,
        .network.timeout_ms = 15000,
        .session.keepalive = 60,
        /* esp-mqtt defaults to a 1024-byte buffer. A full telemetry batch is
         * ~1.1 kB per record, so anything but a one-record batch overran it.
         * That is why the drain stalled at 132 pending on 2026-09-05 while the
         * short status payload kept getting through in the same session. */
        .buffer.size = 4096,
        .buffer.out_size = 12288,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    if (!s_mqtt) return false;
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    xEventGroupClearBits(s_ev, EV_MQTT_UP | EV_MQTT_FAIL);
    if (esp_mqtt_client_start(s_mqtt) != ESP_OK) return false;
    EventBits_t b = xEventGroupWaitBits(s_ev, EV_MQTT_UP | EV_MQTT_FAIL,
                                        pdTRUE, pdFALSE,
                                        pdMS_TO_TICKS(MQTT_CONNECT_TO_MS));
    return (b & EV_MQTT_UP) != 0;
}

static void mqtt_down(void)
{
    if (!s_mqtt) return;
    esp_mqtt_client_stop(s_mqtt);
    esp_mqtt_client_destroy(s_mqtt);
    s_mqtt = NULL;
}

/* QoS 1 publish, blocking until PUBACK -- the ring cursor only advances on ack. */
static bool publish_acked(const char *json)
{
    if (!s_mqtt) return false;
    xEventGroupClearBits(s_ev, EV_PUBACK | EV_MQTT_FAIL);
    int id = esp_mqtt_client_publish(s_mqtt, "v1/devices/me/telemetry",
                                     json, 0, 1, 0);
    if (id < 0) return false;
    s_pending_msg_id = id;
    EventBits_t b = xEventGroupWaitBits(s_ev, EV_PUBACK | EV_MQTT_FAIL,
                                        pdTRUE, pdFALSE,
                                        pdMS_TO_TICKS(PUBLISH_TO_MS));
    return (b & EV_PUBACK) != 0;
}

/* ---- ThingsBoard JSON, same schema the dashboards already use ---- */

/* Returns the length written, or -1 if the payload did not fit. A truncated
 * record is worse than no record: it produces malformed JSON that takes the
 * whole batch down with it. */
static int record_values(const LogRecord *r, char *out, size_t cap)
{
    /* TRIMMED SET (2026-08-15): ~40 datapoints -> 22.
     *
     * Rule applied: publish what you would act on or analyse. Drop what is
     * derivable from other fields, what is only wanted for a post-mortem, and
     * what is structurally constant. Nothing is actually lost -- the 128-byte
     * flash record still carries every field, so a unit can be interrogated
     * after the fact; this only trims what crosses the air.
     *
     * Dropped and why:
     *   bat_mw                 = vbat * ibat, derivable
     *   vsys_mv, vreg_mv,
     *   vindpm_mv              charger internals; post-mortem, not decisions
     * RESTORED 2026-09-04 (the trim went one key too far):
     *   vac2_mv                without it there is no way to tell "no panel
     *                          voltage" from "panel fine, charger refusing" --
     *                          the exact question that could not be answered
     *                          remotely during the 2026-09 discharge test
     *   tdie_c, ts_stat        the two JEITA channels: they distinguish
     *                          "charging stopped because the CELL was hot"
     *                          (ts_stat) from "because the CHARGER was hot"
     *                          (tdie_c). Without them both look identical from
     *                          a dashboard: current fell, nobody knows why.
     *   fault0, fault1         almost always 0; the record keeps them
     *   weather_good, eco_chg  derived heuristics, recomputable from harvest
     *   light_ch1              IR channel; ch0 is the one that means anything
     *   press_mbar             DUPLICATE of p_gas_hpa (same MS5837 reading)
     *   ts_pct                 raw NTC ratio; ts_stat carries the verdict
     *   flow_lpm               VFR constants are wrong for this cell; it is a
     *                          misleading number until the lab calibration
     *   uss_amp_ups/dns, gain  summarised by uss_snr_db (a degrading signal
     *                          shows up there); the record keeps the detail
     *   uss_status             AUTO/BOOT bits; vol_ml resetting says the same
     *   p_atm_hpa, is_const    a compile-time constant, same every sample
     *   wf_praw, wf_traw       the WF280A does not produce usable data
     *
     * uss_tof_us is the MEAN of the two directions, not both. That is the
     * principled split: the mean is the speed-of-sound (composition) signal,
     * and the difference is the flow signal, which uss_dtof_us already carries
     * at far better resolution. Publishing both directions separately sent the
     * same information twice. */
    int n = snprintf(out, cap,
        "\"vbat_mv\":%u,\"ibat_ma\":%d,\"soc_pct\":%u,"
        "\"vbus_mv\":%u,\"ibus_ma\":%d,\"chg_stat\":%u,"
        "\"harvest_mah\":%u,\"solar\":%u,\"usb\":%u,\"motion\":%u,"
        "\"light_ch0\":%u,"
        "\"acc_x_mg\":%d,\"acc_y_mg\":%d,\"acc_z_mg\":%d,"
        "\"temp_c\":%.2f,\"sensor_ok\":%u,"
        "\"vac2_mv\":%u,\"tdie_c\":%.1f,\"ts_stat\":%u",
        r->vbat_mv, r->ibat_ma, r->soc_pct,
        r->vbus_mv, r->ibus_ma, r->chg_stat,
        r->harvest_mah,
        (r->flags & RECF_SOLAR) ? 1 : 0, (r->flags & RECF_USB) ? 1 : 0,
        (r->flags & RECF_MOTION) ? 1 : 0,
        r->light_ch0,
        r->acc_mg[0], r->acc_mg[1], r->acc_mg[2],
        r->temp_cC / 100.0f, r->sensor_ok,
        r->vac2_mv, r->tdie_dC / 10.0f, r->ts_stat);

    /* Ultrasonic: four core values plus SNR as the health indicator. Only when
     * the module answered, so boards without a gas cell burn no quota. */
    if ((r->sensor_ok & 0x10) && n > 0 && (size_t)n < cap) {
        double tof_us = (r->uss_tof_ups_q40 / 1099511.627776 +
                         r->uss_tof_dns_q40 / 1099511.627776) / 2.0;
        n += snprintf(out + n, cap - n,
            ",\"uss_tof_us\":%.4f,\"uss_dtof_us\":%.6f,"
            "\"uss_code\":%u,\"uss_snr_db\":%.1f,\"uss_vol_ml\":%lu",
            tof_us, r->uss_dtof_ps / 1e6f,
            r->uss_code, r->uss_snr_db2 / 2.0f,
            (unsigned long)r->uss_vol_ml);
        /* Capture-quality rate over the whole interval (~300 captures) rather
         * than the single capture uss_code describes. Suppressed when the count
         * is 0 (no delta available) or 0xFFFF (a record written before the
         * field existed -- the ring is 0xFF-filled), so the dashboard never
         * averages a sentinel against a real measurement. */
        if (r->uss_cap_n != 0 && r->uss_cap_n != 0xFFFF && (size_t)n < cap) {
            n += snprintf(out + n, cap - n,
                ",\"uss_cap_n\":%u,\"uss_cap_badcode\":%u,\"uss_cap_badsnr\":%u",
                r->uss_cap_n, r->uss_cap_badcode, r->uss_cap_badsnr);
        }
    }

    /* WF280A raw counts. Re-enabled 2026-09-05: the part answers reliably, and
     * if it sits in a DIFFERENT pressure domain than the MS5837 (ambient rather
     * than the line) it is the live atmospheric reference that dp_hpa currently
     * fakes with a compile-time constant -- which means dp_hpa is presently
     * measuring the weather as much as the gas. No counts->hPa conversion
     * exists (the compensation polynomial is vendor-private), so these go out
     * raw and the map gets fitted from data: log them against p_gas_hpa and see
     * whether they track (same domain, useless) or diverge (usable reference). */
    if ((r->sensor_ok & SOK_WF280A) && n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            ",\"wf_praw\":%lu,\"wf_traw\":%lu",
            (unsigned long)r->wf_praw, (unsigned long)r->wf_traw);

    /* Gas pressure from the MS5837 (it sits in the LINE, not ambient), and the
     * differential against the per-site atmospheric constant in config.h. */
    if ((r->sensor_ok & 0x08) && n > 0 && (size_t)n < cap) {
        float p_gas = r->press_dmbar / 10.0f;
        n += snprintf(out + n, cap - n,
            ",\"p_gas_hpa\":%.2f,\"dp_hpa\":%.2f",
            p_gas, p_gas - P_ATM_CONST_HPA);
    }

    /* snprintf returns what it WOULD have written, so n > cap means we lost
     * bytes somewhere above. Say so rather than emitting broken JSON. */
    if (n < 0 || (size_t)n >= cap) return -1;
    return n;
}

static char s_json[10240];

/* Reference for back-dating: the uptime and boot epoch of the newest record
 * (set from the ctx at the start of every session). */
static uint32_t s_ref_uptime_s;
static uint8_t  s_ref_boot_id;

/* Timestamp for one record.
 *
 * A record logged before the first clock sync carries ts_s = 0 and has to be
 * placed in time at upload. Doing that from its ring position assumes every
 * wake was exactly SAMPLE_INTERVAL_S apart -- which is false in precisely the
 * situations a clockless device is in: critical-battery mode (x6), park mode
 * (3600 s), button wakes and crash reboots. uptime_s is monotonic elapsed time
 * and carries no such assumption, so it goes first.
 *
 * It is only meaningful within one boot epoch (a cold boot restarts uptime at
 * 0 and rolls boot_id), so records from an older epoch keep the position
 * estimate as a last resort -- coarse, but better than a timestamp built on an
 * uptime that has since restarted.
 *
 * Both branches anchor at now_ms, so everything is late by however long this
 * session has been running (minutes at worst); the alternative is baking the
 * session duration into the reference, which buys little and can go negative. */
static int64_t record_ts_ms(const LogRecord *r, int64_t now_ms, uint32_t head)
{
    if (r->ts_s) return (int64_t)r->ts_s * 1000;
    if (r->boot_id == s_ref_boot_id && r->uptime_s <= s_ref_uptime_s)
        return now_ms - (int64_t)(s_ref_uptime_s - r->uptime_s) * 1000;
    return now_ms - (int64_t)(head - 1 - r->seq) * SAMPLE_INTERVAL_S * 1000;
}

/* Sample VBAT under load and keep the minimum. Called straight after a publish,
 * i.e. while the radio is still hot -- the A7672's 2 A bursts are what actually
 * threaten the modem's brown-out limit, and no other sample in the system ever
 * sees them (every logged VBAT is taken with the modem powered down). */
static void note_vbat_under_load(uplink_result_t *res)
{
    uint16_t v = bq_vbat_mv();
    if (v == 0) return;                       /* BQ absent or ADC off */
    if (res->vbat_load_mv == 0 || v < res->vbat_load_mv) res->vbat_load_mv = v;
}

static uint32_t drain(uplink_result_t *res)
{
    uint32_t sent = 0;
    /* Batch size is adaptive, not fixed. A publish can fail for reasons that
     * depend on SIZE -- the broker's message limit, its per-message datapoint
     * limit, a marginal link -- and the old code simply gave up, leaving the
     * ring stalled forever: every later session republished the same oversized
     * batch and failed identically. Halving on failure means the drain finds
     * whatever ceiling actually applies and gets under it, turning a permanent
     * stall into a slower drain. */
    uint32_t batch = BATCH_RECORDS;
    while (flashlog_pending() > 0) {
        esp_task_wdt_reset();
        int64_t now_ms = clock_valid() ? (int64_t)time(NULL) * 1000 : 0;
        uint32_t n = flashlog_pending();
        if (n > batch) n = batch;

        size_t len = 0;
        uint32_t included = 0;
        s_json[len++] = '[';
        for (uint32_t i = 0; i < n; i++) {
            LogRecord r;
            if (!flashlog_peek(i, &r)) continue;
            /* 1536, not 800: the record payload is ~1070 chars now (28 base
             * fields + 9 USS + 4 ToF, two of them 9-digit Q40 integers + 4
             * pressure + 2 WF280A). At 800 it truncated mid-field, the JSON
             * came out malformed, and ThingsBoard silently rejected the whole
             * batch -- the symptom was every sensor key frozen while the
             * separate (short) status payload kept updating. */
            /* STATIC, not on the stack. The main task stack is 4 KB and a
             * 1536-byte automatic here overflowed it -- ESP-IDF caught it as a
             * stack protection fault mid-upload, so the device rebooted every
             * cycle and never sent anything. Static is safe: this loop is
             * single-threaded and the buffer is consumed before the next pass. */
            static char vals[1536];
            if (record_values(&r, vals, sizeof(vals)) < 0) {
                ESP_LOGE(TAG, "record payload truncated -- telemetry dropped");
                continue;
            }
            int w;
            if (now_ms > 0)
                w = snprintf(s_json + len, sizeof(s_json) - len - 2,
                             "%s{\"ts\":%lld,\"values\":{%s}}",
                             included ? "," : "",
                             (long long)record_ts_ms(&r, now_ms, flashlog_head_seq()),
                             vals);
            else
                w = snprintf(s_json + len, sizeof(s_json) - len - 2,
                             "%s{%s}", included ? "," : "", vals);
            if (w < 0 || (size_t)w >= sizeof(s_json) - len - 2) break;
            len += (size_t)w;
            included++;
        }
        s_json[len++] = ']';
        s_json[len] = 0;

        if (included == 0) { flashlog_advance(n); continue; }
        if (!publish_acked(s_json)) {
            /* Broker churn (TB Cloud drops the connection every ~18 s during
             * drains): wait out the client's auto-reconnect, then retry --
             * turns a stopped drain into a slightly slower one. */
            esp_task_wdt_reset();
            xEventGroupWaitBits(s_ev, EV_MQTT_UP, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(15000));
            esp_task_wdt_reset();
            if (!publish_acked(s_json)) {
                if (batch > 1) {
                    batch /= 2;
                    ESP_LOGW(TAG, "publish failed -- retrying with batch=%lu",
                             (unsigned long)batch);
                    continue;          /* same records, smaller payload */
                }
                ESP_LOGW(TAG, "drain stopped at batch=1, %lu pending",
                         (unsigned long)flashlog_pending());
                break;
            }
        }
        flashlog_advance(n);
        note_vbat_under_load(res);
        sent += included;
        if (flashlog_pending() > 0) vTaskDelay(pdMS_TO_TICKS(BATCH_GAP_MS));
    }
    return sent;
}

/* QoS-1 publish that survives broker churn: on failure wait out the client's
 * auto-reconnect, then retry once (2026-08-13 audit: the status record was
 * being eaten by the end-of-session disconnect nearly every time). */
static bool publish_resilient(const char *json)
{
    if (publish_acked(json)) return true;
    esp_task_wdt_reset();
    xEventGroupWaitBits(s_ev, EV_MQTT_UP, pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
    esp_task_wdt_reset();
    return publish_acked(json);
}

static void send_status(const uplink_ctx_t *ctx, const uplink_result_t *res)
{
    const char *transport = (res->sent && res->used_wifi) ? "wifi" :
                            res->sent ? "cell" : "none";
    int64_t now_ms = clock_valid() ? (int64_t)time(NULL) * 1000 : 0;
    static char vals[800];          /* keep it off the 4 KB main stack too */
    snprintf(vals, sizeof(vals),
             "\"rssi_dbm\":%d,\"wifi_rssi_dbm\":%d,\"transport\":\"%s\","
             "\"cereg_stat\":%u,\"backlog\":%lu,\"boot_id\":%u,"
             "\"reset_reason\":%u,\"boot_count\":%u,\"wake_count\":%u,"
             "\"crash_count\":%u,\"vbat_mv\":%u,\"vbat_load_mv\":%u,"
             "\"sun_h\":%u,\"voc_max_mv\":%u,\"fw\":\"" FW_VERSION "\"",
             res->cell_rssi_dbm, res->wifi_rssi_dbm, transport,
             res->cell_reg_stat, (unsigned long)flashlog_pending(), ctx->boot_id,
             ctx->reset_reason, ctx->boot_count, ctx->wake_count,
             ctx->crash_count, ctx->vbat_mv, res->vbat_load_mv,
             ctx->sun_hours, ctx->voc_max_mv);
    /* Appended only on wakes that actually measured. Publishing a placeholder
     * would put zeros in the timeseries and make the dashboard average them
     * against real readings. */
    if (ctx->audit_valid) {
        size_t n = strlen(vals);
        snprintf(vals + n, sizeof(vals) - n,
                 ",\"uss_rail_ua\":%ld,\"uss_rail_se_ua\":%ld,\"base_ua\":%ld",
                 (long)ctx->audit_rail_ua, (long)ctx->audit_se_ua,
                 (long)ctx->audit_base_ua);
    }
    if (now_ms > 0)
        snprintf(s_json, sizeof(s_json), "{\"ts\":%lld,\"values\":{%s}}",
                 (long long)now_ms, vals);
    else
        snprintf(s_json, sizeof(s_json), "{%s}", vals);
    publish_resilient(s_json);
}

/* Parse an RFC 7231 Date header ("Tue, 11 Aug 2026 16:29:42 GMT") to epoch. */
static time_t parse_http_date(const char *s)
{
    static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
    int d, y, hh, mm, ss;
    char mstr[4] = { 0 };
    const char *comma = strchr(s, ',');
    if (!comma) return 0;
    if (sscanf(comma + 1, " %d %3s %d %d:%d:%d", &d, mstr, &y, &hh, &mm, &ss) != 6)
        return 0;
    const char *mp = strstr(mon, mstr);
    if (!mp) return 0;
    int m = (int)(mp - mon) / 3;   /* 0-based month */
    /* days-from-civil (Howard Hinnant), valid for y >= 1970 */
    int yy = y - (m < 2);
    int era = yy / 400;
    int yoe = yy - era * 400;
    int doy = (153 * (m + (m > 1 ? -2 : 10)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = (long)era * 146097 + doe - 719468;
    return (time_t)days * 86400 + hh * 3600 + mm * 60 + ss;
}

/* Fetch trusted time from the TB server's HTTPS Date header. The carrier can
 * intercept UDP NTP; it cannot rewrite a header inside our TLS session. */
static char s_date_hdr[64];

static esp_err_t http_evt(esp_http_client_event_t *e)
{
    if (e->event_id == HTTP_EVENT_ON_HEADER &&
        strcasecmp(e->header_key, "Date") == 0) {
        strncpy(s_date_hdr, e->header_value, sizeof(s_date_hdr) - 1);
    }
    return ESP_OK;
}

static time_t https_trusted_time(void)
{
    s_date_hdr[0] = 0;
    esp_http_client_config_t cfg = {
        .url = "https://eu.thingsboard.cloud/api/v1/ping",
        .cert_pem = isrg_root_pem,
        .timeout_ms = 10000,
        .event_handler = http_evt,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return 0;
    esp_http_client_perform(c);   /* any status is fine; we want the header */
    esp_http_client_cleanup(c);
    return s_date_hdr[0] ? parse_http_date(s_date_hdr) : 0;
}

static void sntp_sync(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
        2, ESP_SNTP_SERVER_LIST("time.google.com", "pool.ntp.org"));
    if (esp_netif_sntp_init(&cfg) == ESP_OK) {
        esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
        esp_netif_sntp_deinit();
    }
    time_t sntp_t = clock_valid() ? time(NULL) : 0;

    /* Cross-check against TLS-protected HTTP time; on disagreement > 2 min the
     * HTTPS source wins (observed 2026-08-11: cellular-synced clock ran ~20-30
     * min fast -- consistent with carrier NTP interception). */
    time_t http_t = https_trusted_time();
    ESP_LOGI(TAG, "clock: sntp=%lld http=%lld (delta %lld s)",
             (long long)sntp_t, (long long)http_t,
             (long long)(sntp_t && http_t ? sntp_t - http_t : 0));
    if (http_t >= CLOCK_MIN && http_t < CLOCK_MAX) {
        if (!sntp_t || llabs((long long)(sntp_t - http_t)) > 120) {
            struct timeval tv = { .tv_sec = http_t };
            settimeofday(&tv, NULL);
            ESP_LOGW(TAG, "clock set from HTTPS Date (SNTP off by %lld s or absent)",
                     sntp_t ? (long long)(sntp_t - http_t) : 0);
        }
    }
    if (clock_valid()) {
        time_t now = time(NULL);
        ESP_LOGI(TAG, "clock now (UTC): %s", ctime(&now));
    }
}

/* ===================== cellular: esp_modem PPP ===================== */

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP || id == IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(s_ev, EV_IP_UP);
}

static void modem_rail(bool on)
{
    gpio_set_level(ECO_PIN_MODEM_PWR_EN, on ? 1 : 0);
}

static void pwrkey_pulse(uint32_t ms)
{
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 0);   /* wait -- inverter: HIGH=asserted */
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 1);
}

/* Wait for EPS registration (+CEREG stat 1/5) with no-signal fail-fast.
 * Returns final stat; fills rssi. */
static uint8_t wait_registration(esp_modem_dce_t *dce, uint32_t timeout_s,
                                 int *rssi_dbm)
{
    uint8_t stat = 0;
    int64_t t0 = esp_timer_get_time();
    bool saw_signal = false;
    char resp[128];

    while ((esp_timer_get_time() - t0) < (int64_t)timeout_s * 1000000) {
        esp_task_wdt_reset();

        int rssi = 99, ber = 0;
        if (esp_modem_get_signal_quality(dce, &rssi, &ber) == ESP_OK && rssi != 99) {
            saw_signal = true;
            *rssi_dbm = -113 + 2 * rssi;
        }
        if (!saw_signal &&
            (esp_timer_get_time() - t0) > (int64_t)NO_SIGNAL_FAILFAST_S * 1000000) {
            ESP_LOGW(TAG, "no signal after %d s -- fail fast", NO_SIGNAL_FAILFAST_S);
            return 0;
        }

        if (esp_modem_at(dce, "AT+CEREG?", resp, 2000) == ESP_OK) {
            char *p = strstr(resp, "+CEREG:");
            int n = 0, st = 0;
            if (p && sscanf(p, "+CEREG: %d,%d", &n, &st) == 2) {
                stat = (uint8_t)st;
                if (st == 1 || st == 5) return stat;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return stat;
}

static bool cell_session(const uplink_ctx_t *ctx, uplink_result_t *res)
{
    /* Power sequencing per the proven Arduino driver: rail, settle, PWRKEY. */
    modem_rail(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    pwrkey_pulse(600);
    for (int i = 0; i < 8; i++) {      /* A7672 UART ready ~8 s after PWRKEY */
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_task_wdt_reset();          /* feed through the settle */
    }

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(SIM_APN);
    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.tx_io_num = MODEM_UART_TX;
    dte_cfg.uart_config.rx_io_num = MODEM_UART_RX;
    dte_cfg.uart_config.baud_rate = 115200;

    esp_netif_config_t ppp_cfg = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *netif = esp_netif_new(&ppp_cfg);
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600,   /* A76xx AT set */
                                             &dte_cfg, &dce_cfg, netif);
    bool ok = false;
    if (!dce) { ESP_LOGE(TAG, "esp_modem init failed"); goto out_netif; }

    if (esp_modem_sync(dce) != ESP_OK) {
        ESP_LOGW(TAG, "modem does not answer AT");
        goto out;
    }

    res->cell_reg_stat = wait_registration(
        dce, ctx->deep_search ? ATTACH_DEEP_S : ATTACH_TIMEOUT_S,
        &res->cell_rssi_dbm);
    if (res->cell_reg_stat != 1 && res->cell_reg_stat != 5) {
        ESP_LOGW(TAG, "not registered (CEREG stat %u, %d dBm)",
                 res->cell_reg_stat, res->cell_rssi_dbm);
        goto out;
    }
    ESP_LOGI(TAG, "registered (stat %u, %d dBm)",
             res->cell_reg_stat, res->cell_rssi_dbm);
    note_vbat_under_load(res);   /* attach bursts are the heaviest load of all */

    xEventGroupClearBits(s_ev, EV_IP_UP);
    if (esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA) != ESP_OK) {
        ESP_LOGW(TAG, "PPP mode switch failed");
        goto out;
    }
    if (!(xEventGroupWaitBits(s_ev, EV_IP_UP, pdTRUE, pdFALSE,
                              pdMS_TO_TICKS(30000)) & EV_IP_UP)) {
        ESP_LOGW(TAG, "PPP: no IP within 30 s");
        goto out;
    }
    ESP_LOGI(TAG, "PPP up");

    sntp_sync();   /* every session: re-sync AND cross-check (clock can be
                    * "valid" yet wrong -- that is exactly the observed bug) */

    if (mqtt_up()) {
        uint32_t sent = drain(res);
        res->sent += sent;
        res->sent ? (res->any_success = true) : 0;
        send_status(ctx, res);   /* always: health must not depend on the drain */
        ok = sent > 0 || flashlog_pending() == 0;
    }
    mqtt_down();

out:
    if (dce) esp_modem_destroy(dce);
out_netif:
    if (netif) esp_netif_destroy(netif);
    /* Rail hard-off is the recovery guarantee -- a wedged modem never survives
     * to the next attempt (the bucket fleet's dead-end, solved in hardware). */
    modem_rail(false);
    return ok;
}

/* ===================== WiFi backup ===================== */

#ifdef WIFI_SSID
static bool wifi_session(const uplink_ctx_t *ctx, uplink_result_t *res)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init) != ESP_OK) return false;

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_set_ps(WIFI_PS_NONE);   /* modem power-save eats packets (dl-2.10) */

    bool ok = false;
    xEventGroupClearBits(s_ev, EV_IP_UP);
    if (esp_wifi_start() != ESP_OK) goto out;
    esp_wifi_connect();
    if (!(xEventGroupWaitBits(s_ev, EV_IP_UP, pdTRUE, pdFALSE,
                              pdMS_TO_TICKS(20000)) & EV_IP_UP)) {
        ESP_LOGW(TAG, "wifi: no IP within 20 s");
        goto out;
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) res->wifi_rssi_dbm = ap.rssi;
    ESP_LOGI(TAG, "wifi up (%d dBm)", res->wifi_rssi_dbm);

    sntp_sync();   /* every session, same reasoning as the cellular path */

    if (mqtt_up()) {
        res->used_wifi = true;
        uint32_t sent = drain(res);
        res->sent += sent;
        if (sent) res->any_success = true;
        send_status(ctx, res);   /* always: health must not depend on the drain */
        ok = sent > 0 || flashlog_pending() == 0;
    }
    mqtt_down();

out:
    esp_wifi_stop();
    esp_wifi_deinit();
    return ok;
}
#endif

/* ===================== entry point ===================== */

/* Held for the duration of an uplink. The modem's UART derives its baud from a
 * clock that DFS moves, and esp_modem does not take its own lock -- without
 * this the first frequency change mid-session corrupts the AT stream. Cheap
 * insurance: a session is ~2 minutes twice a day, and the sampling wakes (which
 * are the ones worth optimising) are unaffected. */
static esp_pm_lock_handle_t s_pm_lock;

static void pm_hold(bool hold)
{
    if (!s_pm_lock &&
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "uplink", &s_pm_lock) != ESP_OK)
        return;
    if (hold) esp_pm_lock_acquire(s_pm_lock);
    else      esp_pm_lock_release(s_pm_lock);
}

uplink_result_t uplink_upload_all(const uplink_ctx_t *ctx)
{
    uplink_result_t res = { 0 };
    pm_hold(true);

    s_ref_uptime_s = ctx->uptime_s;
    s_ref_boot_id  = ctx->boot_id;

    if (!s_ev) {
        s_ev = xEventGroupCreate();
        nvs_flash_init();   /* esp_wifi requires NVS */
        esp_netif_init();
        esp_event_loop_create_default();
        esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, on_ip_event, NULL);
    }

    bool input = ctx->vbat_mv == 0;   /* unknown VBAT (no BQ): don't gate */

    /* Cellular primary, battery-gated (A7672 bursts sag the pack). */
    if (!ctx->skip_cellular && (input || ctx->vbat_mv >= MODEM_MIN_VBAT_MV)) {
        cell_session(ctx, &res);
    } else if (ctx->skip_cellular) {
        ESP_LOGW(TAG, "skipping cellular (previous attempt crashed)");
    } else {
        ESP_LOGW(TAG, "VBAT %u mV below modem floor %d -- cellular skipped",
                 ctx->vbat_mv, MODEM_MIN_VBAT_MV);
    }

#ifdef WIFI_SSID
    if (flashlog_pending() > 0 && (input || ctx->vbat_mv >= WIFI_MIN_VBAT_MV))
        wifi_session(ctx, &res);
#endif

    pm_hold(false);
    res.all_sent = (flashlog_pending() == 0);
    ESP_LOGI(TAG, "%lu records sent%s, %lu pending",
             (unsigned long)res.sent, res.used_wifi ? " (wifi used)" : "",
             (unsigned long)flashlog_pending());
    return res;
}
