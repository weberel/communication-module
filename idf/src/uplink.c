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
#include "driver/uart.h"
#include "bq25792.h"
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
#include "esp_pm.h"   /* reproduction experiment 2026-09-14 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "board.h"
#include "secrets.h"

static const char *TAG = "uplink";

/* Session stopwatch. Zeroed when the modem rail comes up, so every phase
 * number below is directly comparable across sessions and across firmware
 * versions -- and reads as elapsed modem-on time, which is what costs. */
static int64_t s_sess_t0;
static uint32_t sess_ms(void)
{
    return (uint32_t)((esp_timer_get_time() - s_sess_t0) / 1000);
}

/* ISRG Root X1 CA, compiled in from isrg_cert.c (generated from the PEM). */
extern const char isrg_root_pem[];
#define isrg_root_pem_start isrg_root_pem

/* ---- timeouts / policy ---- */
#define MODEM_UART_TX        20
#define MODEM_UART_RX        21

/* Link speed to the A7672.
 *
 * START is what the modem is guaranteed to answer on after a power cycle;
 * AT+IPR is volatile on SIMCom, so every session begins here.
 * FAST is the target once we are talking. 921600 needs R405-R408 at 2.2 k --
 * with the stock 10 k level-shifter pull-ups 230400 is the ceiling and 460800
 * is dead (testing-status.md). Lower FAST to 230400 on an un-reworked board. */
/* Upper bound on how long we wait for the modem to answer AT after PWRKEY.
 * Generous on purpose -- this is a ceiling, not a delay: the poll exits as soon
 * as the modem replies, so a slow boot costs only what it actually costs. */
#define MODEM_READY_TO_MS    15000

#define MODEM_BAUD_START     115200
/* DISABLED (set equal to START) after a measured failure, 2026-09-21.
 *
 * TRIED: 921600, which testing-status.md says this unit's reworked 2.2 k
 * level-shifter pull-ups support. RESULT: "did not stick", and the session then
 * failed at REGISTRATION (reg=0) having burned 37.6 s.
 *
 * WHY THE FALLBACK BELOW CANNOT WORK. AT+IPR SUCCEEDS -- the modem moves to the
 * new rate. If our own re-sync then fails, dropping our UART back to 115200
 * leaves the modem at 921600, i.e. mismatched in the one direction that cannot
 * be talked out of: commanding it back needs the link that is broken. Only a
 * rail power-cycle recovers it, which is why the whole session was lost.
 *
 * AND IT WAS NEVER THE BOTTLENECK. One batch is 8 records ~7 KB = 0.6 s at
 * 115200, yet the measured failure was n_batch=0 with an 18.4 s drain: the
 * first publish transmitted fine and then waited for an ACK that never came.
 * The session's cost is TIMEOUTS (publish 20 s, reconnect 15 s, teardown 18.5 s,
 * modem boot 8.9 s), not throughput. Raise this only with a re-sync that is
 * retried at BOTH rates before giving up, and only if throughput is ever shown
 * to bind. */
#define MODEM_BAUD_FAST      115200
#define ATTACH_TIMEOUT_S     45      /* normal attach budget */
#define ATTACH_DEEP_S        180     /* once-a-day deep search */
#define NO_SIGNAL_FAILFAST_S 25      /* CSQ still 99 after this -> abort attempt */
#define MQTT_CONNECT_TO_MS   30000
#define PUBLISH_TO_MS        20000
/* Back to 8 after the telemetry trim: 22 datapoints per record x 8 = ~176 per
 * publish, comfortably under what made ThingsBoard Cloud drop the connection
 * (that was ~320, at 40 datapoints x 8). Larger batches drain a backlog in
 * fewer round trips, which matters over a slow cellular link. */
#define BATCH_RECORDS        8
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
        .broker.address.uri = MQTT_URI,
        /* PINNED, do not touch. Let's Encrypt moved to the Generation Y root in
         * Jan 2026, so certs now chain to ISRG Root YR -- the SERVER is
         * configured to serve the chain cross-signed by X1 precisely so this
         * file keeps working. X1 is being retired eventually; pin Root YR (or
         * both) then, not now. */
        .broker.verification.certificate = isrg_root_pem_start,
        .credentials.username = MQTT_USERNAME,
        /* The secret goes in the PASSWORD field, not the username: brokers log
         * usernames on every connect. */
        .credentials.authentication.password = MQTT_PASSWORD,
        /* MQTT 5, and this is not cosmetic. On 3.1.1 a broker that REFUSES a
         * publish still returns PUBACK -- there is no not-authorised code --
         * and publish_acked() advances the flash ring cursor on PUBACK. A
         * refused publish would therefore discard records that never arrived.
         * MQTT 5 returns reason code 0x87 and the refusal becomes visible. */
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.timeout_ms = 15000,
        .session.keepalive = 60,
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
    /* Per-device topic. The broker ACL is `pattern write ecotrace/%u/telemetry`,
     * so this must match the authenticated username or the publish is refused
     * -- visibly, now that we speak MQTT 5. */
    int id = esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC, json, 0, 1, 0);
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
     *   vac2_mv, vsys_mv,
     *   vreg_mv, vindpm_mv     charger internals; post-mortem, not decisions
     *   fault0, fault1         almost always 0; the record keeps them
     *   weather_good, eco_chg  derived heuristics, recomputable from harvest
     *   light_ch1              IR channel; ch0 is the one that means anything
     *   press_mbar             DUPLICATE of p_gas_hpa (same MS5837 reading)
     *   tdie_c, ts_pct,
     *   ts_stat                charger thermal diagnostics
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
        "\"harvest_mah\":%u,\"solar\":%u,\"usb\":%u,"
        "\"light_ch0\":%u,"
        "\"acc_x_mg\":%d,\"acc_y_mg\":%d,\"acc_z_mg\":%d,"
        "\"temp_c\":%.2f,\"sensor_ok\":%u",
        r->vbat_mv, r->ibat_ma, r->soc_pct,
        r->vbus_mv, r->ibus_ma, r->chg_stat,
        r->harvest_mah,
        (r->flags & RECF_SOLAR) ? 1 : 0, (r->flags & RECF_USB) ? 1 : 0,
        r->light_ch0,
        r->acc_mg[0], r->acc_mg[1], r->acc_mg[2],
        r->temp_cC / 100.0f, r->sensor_ok);

    /* Supply and charger faults. Restored to telemetry 2026-09-22: these are
     * read into every record already, but the 2026-08-15 trim dropped them to
     * fit a ThingsBoard datapoint limit we no longer have. VSYS is the rail the
     * ESP32 and the module actually brown out on -- without it every reset
     * analysis is inferred from VBAT, which is the wrong node. */
    if (n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            ",\"vsys_mv\":%u,\"fault0\":%u,\"fault1\":%u",
            r->vsys_mv, r->fault0, r->fault1);

    /* Ultrasonic: four core values plus SNR as the health indicator. Only when
     * the module answered, so boards without a gas cell burn no quota. */
    if ((r->sensor_ok & 0x10) && n > 0 && (size_t)n < cap) {
        double tof_us = (r->uss_tof_ups_q40 / 1099511.627776 +
                         r->uss_tof_dns_q40 / 1099511.627776) / 2.0;
        n += snprintf(out + n, cap - n,
            ",\"uss_tof_us\":%.4f,\"uss_dtof_us\":%.6f,"
            "\"uss_code\":%u,\"uss_snr_db\":%.1f,\"uss_vol_ml\":%lu,\"uss_seq\":%d,\"uss_recov\":%u"
            /* RAW TOTALIZER, restored 2026-09-22 now that the module runs
             * PROTO 3 (before that these were identically zero, and the link
             * was hitting a ThingsBoard datapoint throttle that no longer
             * exists).
             *
             * ABSOLUTE running sums, not per-record deltas: diff them
             * server-side to get a window, which is what makes a dropped
             * uplink cost nothing. These are the numbers calibration is fitted
             * against, and the only ones that stay correct when K or the zero
             * offset is changed later -- see the derivation in uss_link.h.
             * Both stay well inside 2^53, so JSON carries them exactly; S0 is
             * the faster of the two at ~3.5e12 per year. */
            ",\"uss_s1\":%lld,\"uss_s0\":%lld,\"uss_skip\":%u",
            tof_us, r->uss_dtof_ps / 1e6f,
            r->uss_code, r->uss_snr_db2 / 2.0f,
            (unsigned long)r->uss_vol_ml,
            /* 0xFFFF = logged before this field existed */
            r->uss_seq == 0xFFFFu ? -1 : (int) r->uss_seq,
            r->uss_recoveries,
            (long long) r->uss_s1, (long long) r->uss_s0, r->uss_tot_skip);
    }

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

static uint32_t drain(uplink_result_t *res)
{
    uint32_t sent = 0;
    while (flashlog_pending() > 0) {
        int64_t t_batch = esp_timer_get_time();

        /* Power doc B2: sample the supply WHILE the modem is transmitting.
         *
         * Taken on the second batch, not the first: by then the link is warm
         * and we are actually pushing data, whereas the first publish can
         * complete before the radio has drawn anything worth measuring. One
         * sample only -- this is a diagnostic, and the I2C read itself costs
         * session time we just spent effort removing. */
        if (res->n_batches == 1 && res->vsys_load_mv == 0) {
            res->vbat_load_mv = bq_vbat_mv();
            res->vsys_load_mv = bq_vsys_mv();
            bq_faults(&res->fault0, &res->fault1);
            ESP_LOGW(TAG, "under load: vbat %u mV, vsys %u mV (idle vbat %u) "
                          "| drop %d mV, fault %02x/%02x",
                     res->vbat_load_mv, res->vsys_load_mv, res->vbat_pre_mv,
                     (int) res->vbat_pre_mv - (int) res->vbat_load_mv,
                     res->fault0, res->fault1);
        }
        esp_task_wdt_reset();
        int64_t now_ms = clock_valid() ? (int64_t)time(NULL) * 1000 : 0;
        uint32_t n = flashlog_pending();
        if (n > BATCH_RECORDS) n = BATCH_RECORDS;

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
                ESP_LOGW(TAG, "drain stopped at batch %u, %lu pending, "
                              "%lu ms into the session",
                         res->n_batches, (unsigned long)flashlog_pending(),
                         (unsigned long)sess_ms());
                res->drain_broke = true;
                break;
            }
        }
        flashlog_advance(n);
        sent += included;
        {
            uint32_t dt = (uint32_t)((esp_timer_get_time() - t_batch) / 1000);
            if (dt > res->t_pub_max_ms) res->t_pub_max_ms = dt;
            res->n_batches++;
        }
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
             "\"crash_count\":%u,\"vbat_mv\":%u,"
             "\"sun_h\":%u,\"voc_max_mv\":%u,\"fw\":\"" FW_VERSION "\","
             "\"uss_rst_cause\":%d,\"uss_lh_starts\":%d,\"uss_lh_uptime\":%d,"
             /* Session phase timing. Cumulative ms of modem-on time; a phase's
              * own cost is the difference from the previous column. */
             "\"t_modem\":%lu,\"t_reg\":%lu,\"t_ppp\":%lu,\"t_sntp\":%lu,"
             "\"t_mqtt\":%lu,\"t_drain\":%lu,\"t_pubmax\":%lu,"
             "\"n_batch\":%u,\"n_sent\":%lu,\"drain_broke\":%u,"
             /* Supply under load -- see power doc B2. vbat_pre is with the
              * modem off; the difference is the sag that sets battery sizing. */
             "\"vbat_pre\":%u,\"vbat_load\":%u,\"vsys_load\":%u,"
             "\"bq_fault0\":%u,\"bq_fault1\":%u",
             res->cell_rssi_dbm, res->wifi_rssi_dbm, transport,
             res->cell_reg_stat, (unsigned long)flashlog_pending(), ctx->boot_id,
             ctx->reset_reason, ctx->boot_count, ctx->wake_count,
             ctx->crash_count, ctx->vbat_mv,
             ctx->sun_hours, ctx->voc_max_mv,
             /* -1 when the health read itself failed, which is itself the
              * datum: it separates "the module restarted" from "we never
              * reached the module at all". */
             ctx->uss_health_valid ? (int) ctx->uss_rst_cause   : -1,
             ctx->uss_health_valid ? (int) ctx->uss_lh_starts   : -1,
             ctx->uss_health_valid ? (int) ctx->uss_lh_uptime_s : -1,
             (unsigned long)res->t_modem_ms, (unsigned long)res->t_reg_ms,
             (unsigned long)res->t_ppp_ms,   (unsigned long)res->t_sntp_ms,
             (unsigned long)res->t_mqtt_ms,  (unsigned long)res->t_drain_ms,
             (unsigned long)res->t_pub_max_ms, res->n_batches,
             (unsigned long)res->sent, res->drain_broke ? 1u : 0u,
             res->vbat_pre_mv, res->vbat_load_mv, res->vsys_load_mv,
             res->fault0, res->fault1);
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

/* Re-sync the clock only when it is actually stale.
 *
 * sntp_sync() costs 5-7 s with the modem powered, and it ran on EVERY session
 * -- including a retry 30 minutes after the last successful sync, and the
 * second and third sessions of the same upload cycle. The RTC keeps time
 * across deep sleep, so the useful rate is daily, not hourly.
 *
 * Still forced whenever the clock is INVALID: record timestamps depend on it,
 * and record_ts_ms() falls back to reconstructing them from uptime, which is
 * exactly the case the cross-check in sntp_sync() exists to catch. */
#define CLOCK_RESYNC_S  (12 * 3600)
RTC_DATA_ATTR static int64_t s_last_clock_sync;

static void sntp_sync(void);

static void maybe_sntp_sync(void)
{
    if (clock_valid() && s_last_clock_sync &&
        llabs((long long)(time(NULL) - s_last_clock_sync)) < CLOCK_RESYNC_S) {
        ESP_LOGI(TAG, "clock fresh (%lld s old) -- skipping sync",
                 (long long)(time(NULL) - s_last_clock_sync));
        return;
    }
    sntp_sync();
    if (clock_valid()) s_last_clock_sync = time(NULL);
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
    /* Baseline with the modem still OFF, so the mid-drain sample has something
     * to be a drop FROM. */
    res->vbat_pre_mv = bq_vbat_mv();

    /* Power sequencing per the proven Arduino driver: rail, settle, PWRKEY. */
    s_sess_t0 = esp_timer_get_time();
    modem_rail(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    pwrkey_pulse(600);

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(SIM_APN);
    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.tx_io_num = MODEM_UART_TX;
    dte_cfg.uart_config.rx_io_num = MODEM_UART_RX;
    dte_cfg.uart_config.baud_rate = MODEM_BAUD_START;

    esp_netif_config_t ppp_cfg = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *netif = esp_netif_new(&ppp_cfg);
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600,   /* A76xx AT set */
                                             &dte_cfg, &dce_cfg, netif);
    bool ok = false;
    if (!dce) { ESP_LOGE(TAG, "esp_modem init failed"); goto out_netif; }

    /* POLL for the modem instead of sleeping a flat 8 s.
     *
     * The old code waited 8 x 1000 ms unconditionally because "A7672 UART ready
     * ~8 s after PWRKEY", then synced once. Measured 2026-09-21: that sleep was
     * 8.9 s of a ~60 s session -- the single largest fixed cost, and the modem
     * is powered for every millisecond of it. Polling returns as soon as it
     * actually answers and keeps the same worst case.
     *
     * Creating the DCE first is safe: esp_modem_new_dev only brings up our UART
     * and the PPP netif, it does not talk to the modem. An AT sent too early
     * just fails and we try again. */
    bool synced = false;
    for (uint32_t waited = 0; waited < MODEM_READY_TO_MS; waited += 250) {
        esp_task_wdt_reset();
        if (esp_modem_sync(dce) == ESP_OK) { synced = true; break; }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (!synced) {
        ESP_LOGW(TAG, "modem does not answer AT after %d ms", MODEM_READY_TO_MS);
        goto out;
    }
    res->t_modem_ms = sess_ms();
    ESP_LOGI(TAG, "modem answered AT at %lu ms", (unsigned long)res->t_modem_ms);

    /* Step the link up. Both ends have to move and the modem goes first: once
     * AT+IPR lands, the modem is no longer listening at the old rate, so our
     * UART must follow immediately. The re-sync afterwards is the proof -- if
     * it fails we back out to MODEM_BAUD_START rather than spending the rest of
     * the session talking into a mismatched link. */
    if (MODEM_BAUD_FAST != MODEM_BAUD_START &&
        esp_modem_set_baud(dce, MODEM_BAUD_FAST) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        uart_set_baudrate(UART_NUM_1, MODEM_BAUD_FAST);
        vTaskDelay(pdMS_TO_TICKS(50));
        if (esp_modem_sync(dce) == ESP_OK) {
            ESP_LOGI(TAG, "link at %d baud", MODEM_BAUD_FAST);
        } else {
            ESP_LOGW(TAG, "%d baud did not stick -- back to %d",
                     MODEM_BAUD_FAST, MODEM_BAUD_START);
            uart_set_baudrate(UART_NUM_1, MODEM_BAUD_START);
            vTaskDelay(pdMS_TO_TICKS(50));
            (void) esp_modem_sync(dce);
        }
    }

    res->cell_reg_stat = wait_registration(
        dce, ctx->deep_search ? ATTACH_DEEP_S : ATTACH_TIMEOUT_S,
        &res->cell_rssi_dbm);
    if (res->cell_reg_stat != 1 && res->cell_reg_stat != 5) {
        ESP_LOGW(TAG, "not registered (CEREG stat %u, %d dBm)",
                 res->cell_reg_stat, res->cell_rssi_dbm);
        goto out;
    }
    res->t_reg_ms = sess_ms();
    ESP_LOGI(TAG, "registered (stat %u, %d dBm) at %lu ms",
             res->cell_reg_stat, res->cell_rssi_dbm,
             (unsigned long)res->t_reg_ms);

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
    res->t_ppp_ms = sess_ms();
    ESP_LOGI(TAG, "PPP up at %lu ms", (unsigned long)res->t_ppp_ms);

    maybe_sntp_sync();
    res->t_sntp_ms = sess_ms();

    if (mqtt_up()) {
        res->t_mqtt_ms = sess_ms();
        uint32_t sent = drain(res);
        res->t_drain_ms = sess_ms();
        res->sent += sent;
        res->sent ? (res->any_success = true) : 0;
        /* AFTER the drain timing is captured, so the status record carries this
         * session's own numbers rather than the previous one's. */
        send_status(ctx, res);
        ok = sent > 0 || flashlog_pending() == 0;
    }
    mqtt_down();

out:
    /* Teardown measured 18.5 s on 2026-09-21 -- as expensive as a failed drain,
     * on a session that sent nothing, and nobody knows which call it is. Stamp
     * each one rather than guess. */
    {
        uint32_t t_a = sess_ms();
        if (dce) esp_modem_destroy(dce);
        uint32_t t_b = sess_ms();
        if (netif) esp_netif_destroy(netif);
        uint32_t t_c = sess_ms();
        ESP_LOGW(TAG, "teardown: mqtt_down->%lu, modem_destroy %lu ms, "
                      "netif_destroy %lu ms",
                 (unsigned long)t_a, (unsigned long)(t_b - t_a),
                 (unsigned long)(t_c - t_b));
        goto rail_off;
    }
out_netif:
    if (netif) esp_netif_destroy(netif);
rail_off:;
    /* Rail hard-off is the recovery guarantee -- a wedged modem never survives
     * to the next attempt (the bucket fleet's dead-end, solved in hardware). */
    modem_rail(false);
    res->t_total_ms = sess_ms();
    /* One line that accounts for the whole session. Phase cost is the gap
     * between consecutive columns; the modem is powered for all of it. */
    ESP_LOGW(TAG, "session: at=%lu reg=%lu ppp=%lu sntp=%lu mqtt=%lu drain=%lu "
                  "total=%lu ms | %u batches, slowest %lu ms, %lu sent%s",
             (unsigned long)res->t_modem_ms, (unsigned long)res->t_reg_ms,
             (unsigned long)res->t_ppp_ms,   (unsigned long)res->t_sntp_ms,
             (unsigned long)res->t_mqtt_ms,  (unsigned long)res->t_drain_ms,
             (unsigned long)res->t_total_ms, res->n_batches,
             (unsigned long)res->t_pub_max_ms, (unsigned long)res->sent,
             res->drain_broke ? ", DRAIN BROKE" : "");
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

    maybe_sntp_sync();

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

/* Held for the duration of an uplink. The modem UART derives its baud from a
 * clock that DFS moves, and esp_modem does not take its own lock -- without
 * this the first frequency change mid-session corrupts the AT stream. Restored
 * verbatim in intent from e86c644: it shipped ALONGSIDE the PM config, so a
 * reproduction without it is not the historical firmware. It also matters for
 * the experiment itself -- an AT-stream corruption can panic the upload, and a
 * panic reboots this board, cycles the sensor rail, resets the module and
 * forces an abs-ToF re-search. That is the exact code-135 signature we are
 * using as the positive result, so leaving this out could fake one. */
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

    res.all_sent = (flashlog_pending() == 0);
    ESP_LOGI(TAG, "%lu records sent%s, %lu pending",
             (unsigned long)res.sent, res.used_wifi ? " (wifi used)" : "",
             (unsigned long)flashlog_pending());
    pm_hold(false);
    return res;
}
