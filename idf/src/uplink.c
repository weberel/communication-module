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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "board.h"
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
        .broker.address.uri = TB_MQTT_URI,
        .broker.verification.certificate = isrg_root_pem_start,
        .credentials.username = TB_ACCESS_TOKEN,
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

static int record_values(const LogRecord *r, char *out, size_t cap)
{
    int32_t bat_mw = (int32_t)r->vbat_mv * r->ibat_ma / 1000;
    int n = snprintf(out, cap,
        "\"vbat_mv\":%u,\"ibat_ma\":%d,\"bat_mw\":%ld,\"soc_pct\":%u,"
        "\"vbus_mv\":%u,\"ibus_ma\":%d,\"vac2_mv\":%u,\"vsys_mv\":%u,"
        "\"vindpm_mv\":%u,\"vreg_mv\":%u,\"chg_stat\":%u,"
        "\"fault0\":%u,\"fault1\":%u,\"harvest_mah\":%u,"
        "\"solar\":%u,\"usb\":%u,\"weather_good\":%u,\"eco_chg\":%u,"
        "\"light_ch0\":%u,\"light_ch1\":%u,"
        "\"acc_x_mg\":%d,\"acc_y_mg\":%d,\"acc_z_mg\":%d,"
        "\"press_mbar\":%.1f,\"temp_c\":%.2f,"
        "\"tdie_c\":%.1f,\"ts_pct\":%.2f,\"ts_stat\":%u,\"sensor_ok\":%u",
        r->vbat_mv, r->ibat_ma, (long)bat_mw, r->soc_pct,
        r->vbus_mv, r->ibus_ma, r->vac2_mv, r->vsys_mv,
        r->vindpm_mv, r->vreg_mv, r->chg_stat,
        r->fault0, r->fault1, r->harvest_mah,
        (r->flags & RECF_SOLAR) ? 1 : 0, (r->flags & RECF_USB) ? 1 : 0,
        (r->flags & RECF_WEATHER) ? 1 : 0, (r->flags & RECF_ECO_CHG) ? 1 : 0,
        r->light_ch0, r->light_ch1,
        r->acc_mg[0], r->acc_mg[1], r->acc_mg[2],
        r->press_dmbar / 10.0f, r->temp_cC / 100.0f,
        r->tdie_dC / 10.0f, r->ts_pct_x100 / 100.0f, r->ts_stat, r->sensor_ok);

    /* Ultrasonic flow + WF280A keys only when the hardware answered this
     * sample -- boards without the gas cell don't burn datapoint quota. */
    if ((r->sensor_ok & 0x10) && n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            /* uss_temp_c is NOT published: USS_ALG_ENABLE_ESTIMATE_TEMPERATURE
             * is false in the gas config (that option derives temperature from
             * ToF for a known gas, which this product does not need), so the
             * field is structurally always 0.00. Publishing it would burn a
             * ThingsBoard datapoint per sample on a constant. */
            ",\"flow_lpm\":%.4f,\"uss_dtof_ns\":%.3f,"
            "\"uss_code\":%u,\"uss_amp_ups\":%u,\"uss_amp_dns\":%u,"
            "\"uss_snr_db\":%.1f,\"uss_gain\":%u,\"uss_vol_ml\":%lu,"
            "\"uss_status\":%u",
            r->uss_flow_ulpm / 1e6f, r->uss_dtof_ps / 1000.0f,
            r->uss_code, r->uss_amp_ups, r->uss_amp_dns,
            r->uss_snr_db2 / 2.0f, r->uss_gain,
            (unsigned long)r->uss_vol_ml, r->uss_status);
    /* Raw absolute ToF, Q40 seconds, unscaled on purpose (see record.h).
     * Sent as integers so no float rounding touches the composition signal. */
    if ((r->sensor_ok & 0x10) && n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            ",\"uss_tof_ups_q40\":%lu,\"uss_tof_dns_q40\":%lu",
            (unsigned long)r->uss_tof_ups_q40, (unsigned long)r->uss_tof_dns_q40);
    /* Absolute pressure PLACEHOLDER -- a compile-time constant, not a reading.
     * The key name and the companion flag both say so; see config.h. */
    if (n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            ",\"p_abs_const_hpa\":%.2f,\"p_abs_is_const\":1", P_ABS_CONST_HPA);
    if ((r->sensor_ok & 0x20) && n > 0 && (size_t)n < cap)
        n += snprintf(out + n, cap - n,
            ",\"wf_praw\":%lu,\"wf_traw\":%lu",
            (unsigned long)r->wf_praw, (unsigned long)r->wf_traw);
    return n;
}

static char s_json[7168];

static int64_t record_ts_ms(const LogRecord *r, int64_t now_ms, uint32_t head)
{
    if (r->ts_s) return (int64_t)r->ts_s * 1000;
    return now_ms - (int64_t)(head - 1 - r->seq) * SAMPLE_INTERVAL_S * 1000;
}

static uint32_t drain(uplink_result_t *res)
{
    uint32_t sent = 0;
    while (flashlog_pending() > 0) {
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
            char vals[800];
            record_values(&r, vals, sizeof(vals));
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
                ESP_LOGW(TAG, "drain stopped, %lu pending",
                         (unsigned long)flashlog_pending());
                break;
            }
        }
        flashlog_advance(n);
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
    char vals[800];
    snprintf(vals, sizeof(vals),
             "\"rssi_dbm\":%d,\"wifi_rssi_dbm\":%d,\"transport\":\"%s\","
             "\"cereg_stat\":%u,\"backlog\":%lu,\"boot_id\":%u,"
             "\"reset_reason\":%u,\"boot_count\":%u,\"wake_count\":%u,"
             "\"crash_count\":%u,\"vbat_mv\":%u,"
             "\"sun_h\":%u,\"voc_max_mv\":%u,\"fw\":\"" FW_VERSION "\"",
             res->cell_rssi_dbm, res->wifi_rssi_dbm, transport,
             res->cell_reg_stat, (unsigned long)flashlog_pending(), ctx->boot_id,
             ctx->reset_reason, ctx->boot_count, ctx->wake_count,
             ctx->crash_count, ctx->vbat_mv,
             ctx->sun_hours, ctx->voc_max_mv);
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

uplink_result_t uplink_upload_all(const uplink_ctx_t *ctx)
{
    uplink_result_t res = { 0 };

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
    return res;
}
