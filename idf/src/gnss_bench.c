/*
 * gnss_bench.c  --  bench test of the A7672E's GNSS and the antenna path.
 * Built only with -DECOTRACE_GNSS_TEST (env:gnsstest). Never returns.
 *
 * What it answers: does the antenna path work? That shows up BEFORE a fix, in
 * the per-satellite C/N0 the receiver reports in NMEA GSV. With an active patch
 * under open sky, a healthy path gives strong satellites at ~35-45 dB-Hz; a
 * lossy one sees satellites weakly (< ~25) or not at all, and never fixes.
 *
 * Sequence (A76XX AT manual V1.09, ch. 24; GNSS app note V1.03):
 *   modem rail + PWRKEY (same sequence as uplink.c) -> AT sync ->
 *   AT+CVAUXS? / AT+CVAUXV?  (active-antenna bias on VDD_AUX, default 3.0 V) ->
 *   AT+CGNSSPWR=1, wait "+CGNSSPWR: READY!" ->
 *   AT+CGNSSPORTSWITCH=0,1 (raw NMEA to the UART we are on) -> AT+CGNSSTST=1 ->
 *   AT+CGPSCOLD (a cold start, which is what every field wake will be) ->
 *   every 5 s a C/N0 summary from GSV + fix state from GGA; every 30 s
 *   AT+CGNSSINFO as the module's own verdict.
 *
 * Leaves the modem powered. Unplug to stop.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "bq25792.h"

static const char *TAG = "gnss";

#define UARTN        UART_NUM_1
#define PIN_TX       20
#define PIN_RX       21
#define BAUD         115200
#define RUN_FOR_S    (20 * 60)     /* summaries stop after this; modem stays on */

static int64_t s_t0;               /* cold start issued */
static double  secs(void) { return (esp_timer_get_time() - s_t0) / 1e6; }

/* ---- line reader ---------------------------------------------------------- */
static char s_line[256];
static int  s_len;

/* Returns a complete line (without CR/LF) or NULL if none within wait_ms. */
static const char *read_line(int wait_ms)
{
    int64_t end = esp_timer_get_time() + (int64_t) wait_ms * 1000;
    uint8_t c;
    do {
        while (uart_read_bytes(UARTN, &c, 1, pdMS_TO_TICKS(10)) == 1) {
            if (c == '\r') continue;
            if (c == '\n') {
                if (s_len == 0) continue;
                s_line[s_len] = 0;
                s_len = 0;
                return s_line;
            }
            if (s_len < (int) sizeof(s_line) - 1) s_line[s_len++] = (char) c;
        }
    } while (esp_timer_get_time() < end);
    return NULL;
}

/* ---- NMEA bookkeeping ------------------------------------------------------ */
typedef struct { char talker[3]; int prn; int snr; } sat_t;
static sat_t s_sats[64];
static int   s_nsat;
static int   s_fixq, s_used;
static char  s_hdop[8];
static bool  s_first_fix_seen;

static void note_sat(const char *talker, int prn, int snr)
{
    for (int i = 0; i < s_nsat; i++)
        if (s_sats[i].prn == prn && !strncmp(s_sats[i].talker, talker, 2)) {
            if (snr > s_sats[i].snr) s_sats[i].snr = snr;
            return;
        }
    if (s_nsat < (int) (sizeof(s_sats) / sizeof(s_sats[0]))) {
        memcpy(s_sats[s_nsat].talker, talker, 2);
        s_sats[s_nsat].talker[2] = 0;
        s_sats[s_nsat].prn = prn;
        s_sats[s_nsat].snr = snr;
        s_nsat++;
    }
}

/* Split a sentence on commas, keeping empty fields. Stops at '*'. */
static int fields(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (; *s && *s != '*' && n < max; s++)
        if (*s == ',') { *s = 0; f[n++] = s + 1; }
    *s = 0;
    return n;
}

static void handle_nmea(const char *line)
{
    char buf[256];
    char *f[32];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    int n = fields(buf, f, 32);
    if (strlen(f[0]) < 6) return;
    const char *talker = f[0] + 1;          /* "GP", "GL", "BD"/"GB", "GA", "GN" */
    const char *type   = f[0] + 3;

    if (!strcmp(type, "GSV")) {
        /* $xxGSV,total,num,inview, then 4 per sat: prn,elev,az,snr */
        for (int i = 4; i + 3 < n; i += 4) {
            if (!*f[i]) continue;
            note_sat(talker, atoi(f[i]), *f[i + 3] ? atoi(f[i + 3]) : 0);
        }
    } else if (!strcmp(type, "GGA") && n > 8) {
        s_fixq = atoi(f[6]);
        s_used = atoi(f[7]);
        strncpy(s_hdop, f[8], sizeof(s_hdop) - 1);
        if (s_fixq > 0 && !s_first_fix_seen) {
            s_first_fix_seen = true;
            ESP_LOGW(TAG, ">>> FIRST FIX after %.0f s (cold start): quality %d, %d sats used, "
                          "HDOP %s", secs(), s_fixq, s_used, s_hdop);
        }
    }
}

static int cmp_desc(const void *a, const void *b)
{
    return ((const sat_t *) b)->snr - ((const sat_t *) a)->snr;
}

static void summary(void)
{
    int tracked = 0, strong = 0;
    qsort(s_sats, s_nsat, sizeof(s_sats[0]), cmp_desc);
    char top[96] = "";
    int  w = 0;
    for (int i = 0; i < s_nsat; i++) {
        if (s_sats[i].snr > 0) tracked++;
        if (s_sats[i].snr >= 30) strong++;
        if (i < 8 && s_sats[i].snr > 0 && w < (int) sizeof(top) - 12)
            w += snprintf(top + w, sizeof(top) - w, " %s%d:%d",
                          s_sats[i].talker, s_sats[i].prn, s_sats[i].snr);
    }
    ESP_LOGI(TAG, "t=%4.0fs  in view %2d, with signal %2d, >=30 dB-Hz %2d | fix q=%d used=%d "
                  "hdop=%s | best:%s",
             secs(), s_nsat, tracked, strong, s_fixq, s_used,
             s_hdop[0] ? s_hdop : "-", top[0] ? top : " none");
    s_nsat = 0;         /* next window starts fresh: GSV repeats every second */
}

/* ---- AT ---------------------------------------------------------------------- */
static void handle_line(const char *l)
{
    if (l[0] == '$') handle_nmea(l);
    else             ESP_LOGI(TAG, "  < %s", l);
}

/* Send a command; print every non-NMEA line until OK/ERROR or timeout. */
static bool at(const char *cmd, int timeout_ms)
{
    ESP_LOGI(TAG, "> %s", cmd);
    uart_write_bytes(UARTN, cmd, strlen(cmd));
    uart_write_bytes(UARTN, "\r", 1);
    int64_t end = esp_timer_get_time() + (int64_t) timeout_ms * 1000;
    while (esp_timer_get_time() < end) {
        const char *l = read_line(50);
        if (!l) continue;
        handle_line(l);
        if (!strcmp(l, "OK"))            return true;
        if (strstr(l, "ERROR") == l)     return false;
    }
    ESP_LOGW(TAG, "  (no final response to %s in %d ms)", cmd, timeout_ms);
    return false;
}

/* Wait for a line containing `what`, printing everything meanwhile. */
static bool wait_for(const char *what, int timeout_ms)
{
    int64_t end = esp_timer_get_time() + (int64_t) timeout_ms * 1000;
    while (esp_timer_get_time() < end) {
        esp_task_wdt_reset();
        const char *l = read_line(100);
        if (!l) continue;
        handle_line(l);
        if (strstr(l, what)) return true;
    }
    return false;
}

void gnss_bench_run(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000));    /* let the USB console attach */
    ESP_LOGW(TAG, "===== GNSS BENCH TEST (env:gnsstest) =====");

    if (bq_begin()) {
        bq_adc_enable(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "VBAT %u mV, VBUS %u mV -- the modem needs a battery; USB alone "
                      "browns it out", bq_vbat_mv(), bq_vbus_mv());
    }

    uart_config_t cfg = {
        .baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UARTN, 8192, 0, 0, NULL, 0);
    uart_param_config(UARTN, &cfg);
    uart_set_pin(UARTN, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* Power sequence copied from uplink.c (proven): rail, settle, PWRKEY. */
    ESP_LOGI(TAG, "modem rail on, PWRKEY pulse");
    gpio_set_level(ECO_PIN_MODEM_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 1);

    bool up = false;
    for (int t = 0; t < 20000 && !up; t += 500) {
        esp_task_wdt_reset();
        uart_write_bytes(UARTN, "AT\r", 3);
        const char *l;
        while ((l = read_line(500)) != NULL) {
            if (!strcmp(l, "OK")) { up = true; break; }
        }
    }
    if (!up) {
        ESP_LOGE(TAG, "modem never answered AT -- check battery / SIM rail. Stopping.");
        for (;;) { esp_task_wdt_reset(); vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    ESP_LOGI(TAG, "modem answers AT");

    at("ATE0", 1000);
    at("AT+SIMCOMATI", 2000);           /* module + firmware revision */
    at("AT+CVAUXS?", 2000);             /* active-antenna bias: must be 1 */
    at("AT+CVAUXV?", 2000);             /* bias voltage, mV (default 3000) */
    at("AT+CGNSSPWR?", 2000);
    /* Ask the module itself whether it has GNSS at all. On a firmware without
     * the GNSS command set, the TEST form (=?) of every GNSS command errors
     * too; on one with it, =? lists the accepted parameters. */
    at("AT+CGMM", 2000);                /* model */
    at("AT+CGMR", 2000);                /* firmware */
    at("AT+CSUB", 2000);                /* firmware sub-version */
    at("AT+CGNSSPWR=?", 2000);
    at("AT+CGPS?", 2000);               /* older SIMCom GPS command set */
    at("AT+CGPSINFO=?", 2000);
    at("AT+CGNSSINFO=?", 2000);

    at("AT+CGNSSPWR=1", 9000);
    if (!wait_for("+CGNSSPWR: READY!", 60000))
        ESP_LOGW(TAG, "no READY URC in 60 s -- continuing anyway (older firmware may "
                      "not send it)");
    at("AT+CGNSSPORTSWITCH=0,1", 9000); /* raw NMEA -> UART (the one we are on) */
    at("AT+CGNSSTST=1", 9000);
    at("AT+CGPSCOLD", 9000);
    s_t0 = esp_timer_get_time();
    ESP_LOGW(TAG, "cold start issued; summaries every 5 s, AT+CGNSSINFO every 30 s");

    int64_t next_sum = esp_timer_get_time() + 5000000;
    int64_t next_info = esp_timer_get_time() + 30000000;
    bool done = false;
    for (;;) {
        esp_task_wdt_reset();
        const char *l = read_line(100);
        if (l) handle_line(l);
        if (done) continue;
        int64_t now = esp_timer_get_time();
        if (now >= next_sum)  { summary(); next_sum += 5000000; }
        if (now >= next_info) { at("AT+CGNSSINFO", 3000); next_info += 30000000; }
        if (secs() > RUN_FOR_S) {
            ESP_LOGW(TAG, "===== %d min done: %s. Modem left on; unplug to stop. =====",
                     RUN_FOR_S / 60, s_first_fix_seen ? "fix obtained" : "NO FIX");
            done = true;
        }
    }
}
