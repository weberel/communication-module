/*
 * ecoTrace datalogger -- ESP-IDF port.
 *
 * Milestone: LOGGING PARITY with the hardware-validated Arduino build
 * (src/datalogger, frozen at dl-2.14). Each wake: full battery/charger state +
 * all three I2C sensors -> one 64 B record into the flash ring -> deep sleep.
 * Includes the field-robustness battery floors + park mode (2026-08-11 spec).
 *
 * NOT yet ported (next milestones, in order):
 *   - uplink: esp_modem PPP + esp-mqtt + mbedTLS (fail-closed, no plaintext),
 *     registration diagnostics, bounded escalation ladder
 *   - error-event log (coredump partition), health telemetry
 *   - OTA with rollback, NVS provisioning, ATECC identity, LP-core sampling
 */
#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_pm.h"   /* reproduction experiment 2026-09-14 -- see sdkconfig */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "board.h"
#include "record.h"
#include "flash_log.h"
#include "ext_flash.h"
#include "bq25792.h"
#include "solar.h"
#include "sensors.h"
#include "uss.h"
#include "i2c_bus.h"
#include "esp_timer.h"
#include "wf280a.h"
#include "uplink.h"

static const char *TAG = "ecotrace";

/* When the switched SENSOR rail came up this wake; the ultrasonic module needs
 * BOARD_SENSOR_BOOT_MS from that point before its I2C slave answers. */
static int64_t s_sensor_up_us;

/* True only when THIS wake actually powered the module up (cold boot, or a
 * recovery power cycle). On a timer wake the SENSOR rail was held on through
 * deep sleep and the module has been running for hours, so there is no boot to
 * wait out. Gating on the FLAG, not on the timestamp: esp_timer_get_time()
 * restarts at 0 every boot, so leaving s_sensor_up_us unstamped makes
 * elapsed_ms ~= the current uptime (~500 ms), which is still < 3000 and waits
 * anyway. That mistake cost a measurement cycle on 2026-09-15. */
static bool s_sensor_cold_up;

/* ---- state surviving deep sleep (and crash reboots) ---- */
#define RTC_STATE_MAGIC 0x8BADF00Du
static RTC_DATA_ATTR uint32_t s_rtc_magic;
static RTC_DATA_ATTR uint32_t s_uptime_s;
static RTC_DATA_ATTR uint32_t s_last_sleep_s;
static RTC_DATA_ATTR uint8_t  s_boot_id;
static RTC_DATA_ATTR uint16_t s_boot_count;
static RTC_DATA_ATTR uint16_t s_wake_count;
static RTC_DATA_ATTR uint16_t s_crash_count;
static RTC_DATA_ATTR uint8_t  s_parked;      /* battery park mode latch */
/* upload scheduling: bounded tempo (2026-08-11 spec) */
static RTC_DATA_ATTR int32_t  s_next_upload_in_s;

/* Module link health for the status record. Plain statics, not RTC_DATA_ATTR:
 * they are re-read every wake and only ever used within the same wake. */
static bool     s_uss_health_valid;
static uint16_t s_uss_rst_cause;
static uint16_t s_uss_lh_starts;
static uint16_t s_uss_lh_uptime_s;

/* Previous raw-totalizer read, kept across deep sleep so the K cross-check can
 * diff consecutive wakes. RTC_DATA_ATTR, not NVS: this is a diagnostic, losing
 * it on a power cycle costs one skipped log line, and it is nowhere near worth
 * a flash write every 5 minutes. */
RTC_DATA_ATTR static int64_t  s_uss_tot_prev_s1;
RTC_DATA_ATTR static int64_t  s_uss_tot_prev_s0;
RTC_DATA_ATTR static uint32_t s_uss_tot_prev_n;
RTC_DATA_ATTR static uint32_t s_uss_tot_prev_ml;
RTC_DATA_ATTR static bool     s_uss_tot_prev_valid;

/* Last uss_seq we published, for stale-frame detection. RTC_DATA_ATTR so the
 * check survives deep sleep -- the fault it looks for lasted ten hours, i.e.
 * ~120 wakes, so a check that reset every wake would never have seen it. */
RTC_DATA_ATTR static uint16_t s_uss_seq_prev;
RTC_DATA_ATTR static bool     s_uss_seq_valid;
RTC_DATA_ATTR static uint16_t s_uss_stale_n;
static RTC_DATA_ATTR uint8_t  s_retries_left;
static RTC_DATA_ATTR uint32_t s_last_deep_uptime;   /* deep search once/day */
static RTC_DATA_ATTR uint8_t  s_upload_inflight;
static RTC_DATA_ATTR uint8_t  s_upload_crashed;

#define UPLOAD_PERIOD_S (12 * 3600)
#define UPLOAD_RETRY_S  (30 * 60)
#define UPLOAD_RETRIES  2

static uint8_t soc_from_voltage(uint16_t vbat_mv)
{
    if (vbat_mv >= 4200) return 100;
    if (vbat_mv <= 3300) return 0;
    return (uint8_t)((vbat_mv - 3300) * 100 / (4200 - 3300));
}

static void read_sample(LogRecord *r, const solar_status_t *sol, bool bq_ok)
{
    memset(r, 0xFF, sizeof(*r));   /* spare bytes stay 0xFF (NOR-friendly) */

    time_t now = time(NULL);
    r->ts_s     = (now >= 1767225600 && now < 2082758400) ? (uint32_t)now : 0;
    r->uptime_s = s_uptime_s;
    r->boot_id  = s_boot_id;

    r->vbat_mv  = bq_vbat_mv();
    r->ibat_ma  = bq_ibat_ma();
    r->vbus_mv  = bq_vbus_mv();
    r->ibus_ma  = bq_ibus_ma();
    r->vac2_mv  = bq_vac2_mv();
    r->vsys_mv  = bq_vsys_mv();
    r->chg_stat = (uint8_t)bq_charge_state();
    bq_faults(&r->fault0, &r->fault1);
    r->soc_pct  = soc_from_voltage(r->vbat_mv);

    r->vindpm_mv   = sol->vindpm_mv;
    r->vreg_mv     = sol->vreg_mv;
    r->harvest_mah = sol->harvest_today_mah;
    r->flags = (sol->solar_present ? RECF_SOLAR   : 0) |
               (sol->usb_present   ? RECF_USB     : 0) |
               (sol->weather_good  ? RECF_WEATHER : 0) |
               (sol->eco_target    ? RECF_ECO_CHG : 0);

    /* Log what is actually on the bus. Cheap (absent devices NACK immediately)
     * and it turns "the reading is zero" into "that chip is not there", which
     * are entirely different faults. */
    eco_i2c_scan();

    r->sensor_ok = bq_ok ? 0x01 : 0x00;


    uint16_t c0, c1;
    if (ltr303_sample(&c0, &c1)) {
        r->light_ch0 = c0; r->light_ch1 = c1;
        r->sensor_ok |= 0x02;
    } else { r->light_ch0 = r->light_ch1 = 0; }

    int16_t ax, ay, az;
    if (sc7a20_sample(&ax, &ay, &az)) {
        r->acc_mg[0] = ax; r->acc_mg[1] = ay; r->acc_mg[2] = az;
        r->sensor_ok |= 0x04;
    } else {
        r->acc_mg[0] = r->acc_mg[1] = r->acc_mg[2] = 0;
    }

    float mbar, degc;
    if (ms5837_sample(&mbar, &degc)) {
        r->press_dmbar = (uint16_t)(mbar * 10.0f + 0.5f);
        r->temp_cC     = (int16_t)(degc * 100.0f + (degc >= 0 ? 0.5f : -0.5f));
        r->sensor_ok |= 0x08;
    } else {
        r->press_dmbar = 0;
        r->temp_cC     = 0;
    }

    /* Charger thermal diagnostics: die temp, battery NTC, and the charger's
     * own JEITA verdict -- so an afternoon charge lockout names itself. */
    r->tdie_dC     = bq_tdie_dC();
    r->ts_pct_x100 = bq_ts_pct_x100();
    r->ts_stat     = bq_ts_stat();

    /* Ultrasonic flow module (MSP430FR6043 I2C slave) + its WF280A pressure
     * sensor, both on the shared bus. Absent on boards without the gas cell:
     * each costs one NACK and the fields stay zero. */
    /* Wait out the module's boot before the first transaction. The on-board
     * sensor reads above have already burned some of it, so only the remainder
     * costs anything. Without this we NACK a module that is merely still
     * booting, then waste a power cycle on it. */
    {
        int64_t elapsed_ms = (esp_timer_get_time() - s_sensor_up_us) / 1000;
        if (s_sensor_cold_up && elapsed_ms < BOARD_SENSOR_BOOT_MS)
            vTaskDelay(pdMS_TO_TICKS(BOARD_SENSOR_BOOT_MS - elapsed_ms));
    }

    uss_result_t u;
    /* One recovery attempt: if the module does not answer, power-cycle the
     * SENSOR rail and try once more. This is the whole point of the gated rail
     * -- a wedged or half-powered slave is otherwise a site visit, and a
     * half-powered slave is a real state (its ESD clamps back-feed from an
     * idle-high bus, so the rail must be dropped with SDA/SCL held low).
     * Costs ~1 s and only on the failing path; a healthy node never sees it. */
    /* Keep the module in autonomous 1 Hz mode. Idempotent, and re-asserting it
     * every wake is what makes it recover by itself if the module rebooted. */
    (void) uss_start_auto(USS_AUTO_PERIOD_S);

    bool uss_ok = uss_sample(&u);

    /* RECOVERY LADDER -- cheapest rung first, rail cycle LAST.
     *
     * This used to be a single rung: one failed sample went straight to
     * board_sensor_power_cycle(). That is the most destructive action available
     * and it was the FIRST response. Every one of those reboots reset the
     * module's abs-ToF lock, and a fresh search on a ~1 % lobe margin is close
     * to a coin flip -- roughly one in four came back locked to the wrong lobe
     * and then TRACKED it, producing code-135 until something reset it again.
     * The reboots also reset the volume totalizer. So a single glitched byte
     * could cost the running total and corrupt measurements for minutes.
     *
     * The rungs, in order:
     *   0. the transaction retry inside uss.c (3 attempts) -- already tried
     *      before we get here;
     *   1. read link health. If the module answers THIS, it is alive and on the
     *      bus, so a power cycle would be pure damage. Just re-assert AUTO;
     *   2. soft reset -- resets the slave state machine without dropping the
     *      rail, so the totalizer survives (it is persisted in FRAM now);
     *   3. rail cycle, only once nothing else answered. */
    if (!uss_ok) {
        uss_health_t h;
        bool alive = uss_read_health(&h);

        if (alive) {
            ESP_LOGW(TAG, "USS sample failed but link healthy "
                          "(starts=%u rst=0x%04X up=%us) -- re-asserting AUTO",
                     h.starts, h.rst_cause, h.uptime_s);
            (void) uss_start_auto(USS_AUTO_PERIOD_S);
            uss_ok = uss_sample(&u);
        }

        if (!uss_ok) {
            ESP_LOGW(TAG, "USS: soft reset");
            if (uss_soft_reset()) {
                vTaskDelay(pdMS_TO_TICKS(USS_SOFT_RESET_SETTLE_MS));
                (void) uss_start_auto(USS_AUTO_PERIOD_S);
                uss_ok = uss_sample(&u);
            }
        }

        if (!uss_ok) {
            ESP_LOGW(TAG, "USS still silent, power-cycling the sensor rail");
            /* 800 ms, not 300: the SENSOR rail is a high-side switch with no
             * bleed resistor, so when it opens the node is left floating and the
             * decoupling caps discharge only through the slave's own quiescent
             * draw. Confirmed on the bench 2026-08-15 with an LED across the
             * header -- it fades rather than switching off. Too short a window
             * means no power-on reset at all, which defeats the purpose. */
            board_sensor_power_cycle(800);
            /* The module has just rebooted, so STATUS.AUTO is clear -- restart
             * autonomous mode before sampling, or it would stay in one-shot
             * until the next wake and the totalizer would never accumulate. */
            (void) uss_start_auto(USS_AUTO_PERIOD_S);
            uss_ok = uss_sample(&u);
            ESP_LOGW(TAG, "USS after power cycle: %s",
                     uss_ok ? "recovered" : "still silent");
        }
    }
    /* Read link health every wake, pass or fail.
     *
     * On a FAILED wake it says whether we reached the module at all. On a
     * PASSING wake it is the baseline that makes the failing one readable: a
     * drop in uptime_s means the module restarted between wakes, and delta
     * starts tells us how many address matches it actually saw. Without a
     * healthy reference the failure numbers mean nothing on their own. */
    {
        uss_health_t h;
        s_uss_health_valid = uss_read_health(&h);
        if (s_uss_health_valid) {
            s_uss_rst_cause   = h.rst_cause;
            s_uss_lh_starts   = h.starts;
            s_uss_lh_uptime_s = h.uptime_s;
        }
    }

    if (uss_ok) {
        r->uss_flow_ulpm = u.flow_ulpm;
        r->uss_dtof_ps   = u.dtof_ps;
        r->uss_temp_cC   = u.temp_cC;
        r->uss_amp_ups   = u.amp_ups;
        r->uss_amp_dns   = u.amp_dns;
        r->uss_code      = u.code;
        r->uss_gain      = u.gain;
        r->uss_snr_db2   = u.snr_db2;
        r->uss_status    = u.status;
        r->uss_seq       = u.seq;
        r->uss_recoveries = u.recoveries;

        /* WEDGED READ PATH. uss_code cannot see this: during the 2026-09-22
         * event every USS field was frozen on one frame for ten hours while the
         * code byte stayed 122 (valid measurement). uss_seq is the only field
         * that separates "measured again, same answer" from "handed us the same
         * answer again" -- the module increments it per capture, so an
         * unchanged seq means no new measurement was produced at all.
         *
         * Counted and reported, deliberately not acted on: the cause is not
         * understood, and the escalation that suggests itself (uss_soft_reset,
         * which preserves the FRAM totalizer) would be an unproven remedy for
         * an unproven diagnosis. */
        if (s_uss_seq_valid && u.seq == s_uss_seq_prev) {
            if (s_uss_stale_n != 0xFFFFu) s_uss_stale_n++;
            ESP_LOGE(TAG, "USS STALE: seq %u unchanged since the last record "
                          "(code=%u claims valid) -- %u consecutive",
                     u.seq, u.code, s_uss_stale_n);
        } else {
            s_uss_stale_n = 0;
        }
        s_uss_seq_prev  = u.seq;
        s_uss_seq_valid = true;
        r->uss_vol_ml    = u.vol_ml;
        r->uss_tof_ups_q40 = u.tof_ups_q40;
        r->uss_tof_dns_q40 = u.tof_dns_q40;
        r->sensor_ok |= 0x10;

        /* Raw totalizer. A module still on PROTO 2 simply has nothing here, so
         * this failing is not an error -- uss_vol_ml above stays authoritative
         * until both sides are flashed. */
        uss_totals_t tot;
        if (uss_read_totalizer(&tot)) {
            r->uss_s1        = tot.s1;
            r->uss_s0        = tot.s0;
            r->uss_tot_skip  = tot.skip;
            r->uss_tot_flags = tot.flags;

            /* Cross-check against the calibrated path while BOTH totalizers
             * run. The geometry constant K is not known yet -- calibration
             * against the MFC is what determines it -- so derive it here from
             * the validated VOL_ML path and let it fall out of the soak
             * instead of guessing a number and fitting the data to it.
             *
             *     K [uL per Q24-count] = d(vol_uL) / (dS1 / 2^24)
             *
             * A K that holds steady across records IS the evidence that the raw
             * path reproduces the calibrated one; a K that drifts with flow or
             * temperature says the two disagree and the raw path is not yet
             * trustworthy. This is a diagnostic, not a control path -- nothing
             * downstream consumes it. */
            if (s_uss_tot_prev_valid && tot.n != s_uss_tot_prev_n) {
                int64_t ds1 = tot.s1 - s_uss_tot_prev_s1;
                int64_t ds0 = tot.s0 - s_uss_tot_prev_s0;
                double  dml = (double) u.vol_ml - (double) s_uss_tot_prev_ml;
                double  q24 = (double) ds1 / 16777216.0;
                ESP_LOGI(TAG, "USS raw: dS1=%lld dS0=%lld skip=%u  "
                              "K_est=%.6g uL/count (dVOL=%.0f mL)",
                         (long long) ds1, (long long) ds0, tot.skip,
                         (q24 != 0.0) ? (dml * 1000.0 / q24) : 0.0, dml);
            }
            s_uss_tot_prev_s1 = tot.s1;
            s_uss_tot_prev_s0 = tot.s0;
            s_uss_tot_prev_n  = tot.n;
            s_uss_tot_prev_ml = u.vol_ml;
            s_uss_tot_prev_valid = true;
        }
        ESP_LOGI(TAG, "USS ok: code=%u seq=%u dtof=%ld ps tof_ups=%lu tof_dns=%lu q40 "
                      "(%.1f/%.1f us) amp=%u/%u snr=%.1f dB gain=%u vol=%lu mL st=0x%02X",
                 u.code, u.seq, (long) u.dtof_ps,
                 (unsigned long) u.tof_ups_q40, (unsigned long) u.tof_dns_q40,
                 u.tof_ups_q40 / 1099511.627776, u.tof_dns_q40 / 1099511.627776,
                 u.amp_ups, u.amp_dns, u.snr_db2 / 2.0f, u.gain,
                 (unsigned long) u.vol_ml, u.status);
    } else {
        r->uss_flow_ulpm = r->uss_dtof_ps = 0;
        r->uss_temp_cC = 0;
        r->uss_amp_ups = r->uss_amp_dns = 0;
        r->uss_code = r->uss_gain = r->uss_snr_db2 = r->uss_status = 0;
        r->uss_seq = 0;
        r->uss_recoveries = 0;
        r->uss_vol_ml = 0;
        r->uss_tof_ups_q40 = r->uss_tof_dns_q40 = 0;
        r->uss_s1 = r->uss_s0 = 0;
        r->uss_tot_skip = 0;
        r->uss_tot_flags = 0;
    }

    uint32_t praw, traw;
    uint8_t  wst;
    if (wf280a_sample(&praw, &traw, &wst)) {
        r->wf_praw = praw;
        r->wf_traw = traw;
        r->sensor_ok |= 0x20;
    } else {
        r->wf_praw = r->wf_traw = 0;
    }
}

void app_main(void)
{
    esp_reset_reason_t why = esp_reset_reason();
    bool timer_wake  = board_woke_from_timer();
    bool button_wake = board_woke_from_button();
    bool cold        = (s_rtc_magic != RTC_STATE_MAGIC);
    bool crashed     = !cold && !timer_wake && !button_wake;

    /* Runtime-enforce the long WDT: sdkconfig regeneration silently reverted
     * TIMEOUT_S to 5 s once (idf-0.5/0.6 boot-looped on the modem's 8 s settle).
     * Reconfigure defensively, then subscribe this task. */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 120000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    if (esp_task_wdt_reconfigure(&wdt_cfg) != ESP_OK)
        esp_task_wdt_init(&wdt_cfg);
    esp_task_wdt_add(NULL);

    /* Tell the bootloader this image works, so it stops being a rollback
     * candidate.
     *
     * partitions.csv has ota_0 and ota_1 and NO factory slot, and
     * sdkconfig.defaults sets CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y. Without
     * this call an image that panics before marking itself valid is ROLLED BACK
     * to the other slot -- and on 2026-09-11 that slot still held a February
     * 2025 Arduino build, which the board then ran for hours while every
     * `esptool write_flash 0x10000` reported "Hash of data verified". The write
     * was fine; it just went to a slot the bootloader had stopped booting.
     *
     * Placed AFTER the task WDT is armed on purpose: a genuine boot loop that
     * hangs before this point still gets rolled back, which is the behaviour
     * rollback exists for. */
    esp_ota_mark_app_valid_cancel_rollback();

    board_init();

    /* Raise the switched SENSOR rail (J404) BEFORE the first I2C transaction.
     *
     * This is not just about powering the ultrasonic module: an UNPOWERED board
     * on the shared bus drags SDA and SCL down through its ESD clamps and kills
     * the bus for everyone. Measured 2026-08-15 with the rail left off -- the
     * BQ25792 on our own always-on rail was not found either, every I2C call
     * returned "clear bus failed", and the whole record came back zeros.
     *
     * So the rail must be up whenever the bus is used, and it is only dropped
     * in deep sleep (where nothing is talking anyway). That still leaves the
     * remote-recovery path intact via board_sensor_power_cycle(). */
    board_sensor_power(true);
    /* Stamp this ONLY on a cold boot.
     *
     * read_sample() waits out BOARD_SENSOR_BOOT_MS (3 s) from this timestamp
     * before the first USS transaction, to avoid NACKing a module that is still
     * booting. But the module only boots when the rail actually comes up, and
     * board_deep_sleep() deliberately leaves the SENSOR rail ON ("dropping it
     * does not save power, it costs power" -- the 4k7 pull-ups back-feed the
     * sleeping board). So on a timer wake the module has been running for hours:
     * measured lh_uptime of 9.8 h across many wakes.
     *
     * Stamping unconditionally therefore made EVERY wake wait 3 s for a boot
     * that never happened. Measured 2026-09-15 on a JS220: 2820 ms of a ~3500 ms
     * wake, at ~16 mA, 288 times a day -- 45 of the 59 mA*s per wake, i.e. ~76 %
     * of the wake energy and ~3.6 mAh/day on a 20 mAh/day node.
     *
     * s_sensor_up_us is a plain static (not RTC_DATA_ATTR), so on a warm wake it
     * stays 0 and the elapsed check trivially passes -- no wait. The recovery
     * path is unaffected: board_sensor_power_cycle() does its own
     * vTaskDelay(BOARD_SENSOR_BOOT_MS) after raising the rail. */
    s_sensor_up_us   = esp_timer_get_time();
    s_sensor_cold_up = cold;

#ifdef ECOTRACE_RAIL_BLINK
    /* Bench helper: does GPIO14 actually switch the SENSOR rail? Put an LED
     * across the J404 header's VCC/GND and watch. Never returns, so the board
     * stays awake and enumerated. */
    ESP_LOGW(TAG, "RAIL BLINK MODE: toggling GPIO14 (SENSOR rail) 1 s on / 1 s off");
    for (;;) {
        board_sensor_power(true);
        ESP_LOGI(TAG, "SENSOR rail ON  (GPIO14 high)");
        vTaskDelay(pdMS_TO_TICKS(1000));
        board_sensor_power(false);
        ESP_LOGI(TAG, "SENSOR rail OFF (GPIO14 low)");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_task_wdt_reset();
    }
#endif

    if (cold) vTaskDelay(pdMS_TO_TICKS(1500));   /* let USB console enumerate */
    ESP_LOGI(TAG, "ecoTrace datalogger %s (ESP-IDF port)", FW_VERSION);

    if (cold) {
        s_rtc_magic        = RTC_STATE_MAGIC;
        s_uptime_s         = 0;
        s_last_sleep_s     = 0;
        s_boot_id          = (uint8_t)esp_random();
        s_boot_count       = 0;
        s_wake_count       = 0;
        s_crash_count      = 0;
        s_parked           = 0;
        s_next_upload_in_s = 0;   /* first boot phones home immediately */
        s_retries_left     = UPLOAD_RETRIES;
        s_last_deep_uptime = 0;
        s_upload_inflight  = 0;
        s_upload_crashed   = 0;
        solar_reset();
        ESP_LOGI(TAG, "cold boot (id %u, reset reason %d)", s_boot_id, (int)why);
    } else if (crashed) {
        s_crash_count++;
        if (s_upload_inflight) {           /* the upload killed us */
            s_upload_crashed  = 1;
            s_upload_inflight = 0;
        }
        if (s_next_upload_in_s <= 0) s_next_upload_in_s = UPLOAD_RETRY_S;
        ESP_LOGW(TAG, "recovered from abnormal reset (reason %d, %u in a row)",
                 (int)why, s_crash_count);
    } else {
        s_wake_count++;
        if (timer_wake) {
            s_uptime_s         += s_last_sleep_s;
            s_next_upload_in_s -= (int32_t)s_last_sleep_s;
        }
        if (button_wake) ESP_LOGI(TAG, "button wake -- sample + upload now");
    }
    s_boot_count++;

    /* Charger first: park-mode decision needs VBAT before we touch anything. */
    bool bq_ok = bq_begin();
    if (bq_ok) bq_adc_enable(true);
    uint16_t vbat = bq_ok ? bq_vbat_mv() : 0;

    /* Battery park mode: below PARK_VBAT_MV (and nothing plugged in) do the
     * absolute minimum -- no flash writes, no sensors, long sleeps -- until the
     * pack recovers past the hysteresis threshold. Brownout mid-flash-write is
     * the failure this avoids. */
    bool input_present = bq_ok && (bq_ac1_present() || bq_ac2_present());

    /* Power management: frequency scaling 40-80 MHz, light sleep OFF. */
    esp_pm_config_t pm = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,                    /* XTAL */
        /* NEVER true. Automatic light sleep is what broke the I2C link to the
         * module: measured 2026-09-14, 0/11 records failed with it off and 4/8
         * with it on (p = 0.018), every failure losing exactly the two devices
         * on the switched sensor rail. It was worth +0.1 uA. Frequency scaling
         * on its own was clean across all 11 and is kept.
         *
         * The sdkconfig also omits CONFIG_FREERTOS_USE_TICKLESS_IDLE, so
         * esp_pm_configure() would reject light sleep even if this were true.
         * Two independent guarantees, on purpose. */
        .light_sleep_enable = false,
    };
    esp_err_t pm_err = esp_pm_configure(&pm);
    if (pm_err != ESP_OK)
        ESP_LOGW(TAG, "esp_pm_configure failed: %s", esp_err_to_name(pm_err));
    else
        ESP_LOGI(TAG, "PM: 40-80 MHz DFS, light sleep permanently off");
    if (bq_ok && !input_present) {
        if (s_parked && vbat < PARK_RESUME_VBAT_MV) {
            ESP_LOGW(TAG, "parked (VBAT %u mV) -- sleeping %d s", vbat, PARK_SLEEP_S);
            bq_adc_disable();
            s_last_sleep_s = PARK_SLEEP_S;
            board_deep_sleep(PARK_SLEEP_S);
        }
        if (!s_parked && vbat > 0 && vbat < PARK_VBAT_MV) {
            ESP_LOGW(TAG, "entering park mode (VBAT %u mV < %d)", vbat, PARK_VBAT_MV);
            s_parked = 1;
            bq_adc_disable();
            s_last_sleep_s = PARK_SLEEP_S;
            board_deep_sleep(PARK_SLEEP_S);
        }
        s_parked = 0;   /* recovered (or never parked) */
    }

    if (!flashlog_begin(cold || crashed))
        ESP_LOGE(TAG, "SPI flash not found -- samples will be lost!");

    solar_status_t sol = { 0 };
    if (bq_ok) {
        bq_ibat_sense(true);
        bq_configure_charging(CHARGE_CURRENT_MA, 0, 0);
        bq_enable_acdrv1(true);
        bq_enable_acdrv2(true);
        /* Local calendar day for harvest/sun rollover once the clock is valid;
         * falls back to a 24 h counter before the first sync. */
        time_t now = time(NULL);
        int32_t day_num = (now >= 1767225600 && now < 2082758400)
                        ? (int32_t)((now + TZ_OFFSET_MIN * 60) / 86400) : -1;
        sol = solar_on_wake(SAMPLE_INTERVAL_S, day_num);
    } else {
        ESP_LOGE(TAG, "BQ25792 not found -- battery data will be zero");
    }

    LogRecord r;
    read_sample(&r, &sol, bq_ok);
    flashlog_append(&r);

#if ECO_FAKE_BACKLOG > 0
    /* ---- DEBUG: synthetic backlog (config.h ECO_FAKE_BACKLOG) -------------
     * Re-append the record we just built until the log holds the target depth.
     * Same record size and same JSON length as the real thing, which is what
     * the drain actually costs; only the timestamps differ, back-dated one
     * sample interval apart so the server sees distinct samples instead of
     * collapsing them onto one.
     *
     * Bounded rather than while(pending<target): if an append ever fails this
     * must not spin with the watchdog running. */
    {
        uint32_t want = ECO_FAKE_BACKLOG;
        for (uint32_t i = 1; i <= want && flashlog_pending() < want; i++) {
            LogRecord f = r;
            if (r.ts_s) f.ts_s = r.ts_s - i * SAMPLE_INTERVAL_S;
            if (!flashlog_append(&f)) break;
        }
        ESP_LOGW(TAG, "DEBUG fake backlog: %lu pending",
                 (unsigned long)flashlog_pending());
    }
#endif
    ESP_LOGI(TAG,
        "seq %lu: VBAT=%umV IBAT=%+dmA SOC=%u%% | in:%s%s VINDPM=%umV IBUS=%dmA | "
        "harvest %umAh (prev %u) weather=%s target=%umV | lux0=%u acc=[%d,%d,%d] "
        "P=%.1fmbar T=%.2fC | pending %lu",
        (unsigned long)r.seq, r.vbat_mv, r.ibat_ma, r.soc_pct,
        sol.usb_present ? " USB" : "", sol.solar_present ? " solar" : "",
        r.vindpm_mv, r.ibus_ma,
        sol.harvest_today_mah, sol.harvest_prev_mah,
        sol.weather_good ? "good" : "bad", r.vreg_mv,
        r.light_ch0, r.acc_mg[0], r.acc_mg[1], r.acc_mg[2],
        r.press_dmbar / 10.0f, r.temp_cC / 100.0f,
        (unsigned long)flashlog_pending());

    /* Decode sensor_ok by name. A zero reading and an absent chip look
     * identical in the summary above; this line separates them. */
    ESP_LOGI(TAG, "sensor_ok=0x%02X  BQ:%c LTR303:%c SC7A20:%c MS5837:%c USS:%c WF280A:%c",
             r.sensor_ok,
             (r.sensor_ok & 0x01) ? 'y' : 'N', (r.sensor_ok & 0x02) ? 'y' : 'N',
             (r.sensor_ok & 0x04) ? 'y' : 'N', (r.sensor_ok & 0x08) ? 'y' : 'N',
             (r.sensor_ok & 0x10) ? 'y' : 'N', (r.sensor_ok & 0x20) ? 'y' : 'N');

    /* Upload if due (or the button asked). Tempo is bounded: 2 quick retries,
     * then only the 12 h schedule -- an outage costs ~2 short attempts a day
     * while everything banks in flash. */
    if (button_wake) s_next_upload_in_s = 0;
    if (s_next_upload_in_s <= 0) {
        uplink_ctx_t ctx = {
            .boot_id      = s_boot_id,
            .reset_reason = (uint8_t)why,
            .boot_count   = s_boot_count,
            .wake_count   = s_wake_count,
            .crash_count  = s_crash_count,
            .vbat_mv      = r.vbat_mv,
            .deep_search  = (s_uptime_s - s_last_deep_uptime) >= 86400,
            .skip_cellular = s_upload_crashed != 0,
            .sun_hours    = sol.sun_hours,
            .voc_max_mv   = sol.voc_max_mv,
            .uptime_s     = s_uptime_s,
            .uss_health_valid = s_uss_health_valid,
            .uss_rst_cause    = s_uss_rst_cause,
            .uss_lh_starts    = s_uss_lh_starts,
            .uss_lh_uptime_s  = s_uss_lh_uptime_s,
        };
        if (ctx.deep_search) s_last_deep_uptime = s_uptime_s;

        s_upload_inflight = 1;
        uplink_result_t u = uplink_upload_all(&ctx);
        s_upload_inflight = 0;
        s_upload_crashed  = 0;   /* one-shot: cellular is primary again */

        if (u.all_sent) {
            s_next_upload_in_s = UPLOAD_PERIOD_S;
            s_retries_left     = UPLOAD_RETRIES;
        } else if (s_retries_left > 0) {
            s_retries_left--;
            s_next_upload_in_s = UPLOAD_RETRY_S;
        } else {
            s_next_upload_in_s = UPLOAD_PERIOD_S;
            s_retries_left     = UPLOAD_RETRIES;
        }
    }

    if (bq_ok) {
        bq_adc_disable();
        bq_ibat_sense(false);   /* EN_IBAT off across sleep (quiescent) */
    }
    flashlog_sleep();

    uint32_t sleep_s = SAMPLE_INTERVAL_S;
    if (r.vbat_mv && r.vbat_mv < CRITICAL_VBAT_MV &&
        !sol.usb_present && !sol.solar_present)
        sleep_s = SAMPLE_INTERVAL_S * CRITICAL_INTERVAL_MULT;

    s_crash_count  = 0;   /* wake completed cleanly */
    s_last_sleep_s = sleep_s;

    /* Repeat the USS state right before sleeping. The USB CDC takes several
     * seconds to enumerate after a wake, so the sampling log at the top of the
     * cycle is unobservable on a console that attaches mid-wake -- this recap
     * always lands. */
    ESP_LOGI(TAG, "USS recap: st=0x%02X%s code=%u vol=%lu mL tof=%.2f/%.2f us dtof=%.4f us",
             r.uss_status, (r.uss_status & 0x08) ? " AUTO" : "", r.uss_code,
             (unsigned long)r.uss_vol_ml,
             r.uss_tof_ups_q40 / 1099511.627776,
             r.uss_tof_dns_q40 / 1099511.627776,
             r.uss_dtof_ps / 1e6f);

    ESP_LOGI(TAG, "sleeping %lu s", (unsigned long)sleep_s);
    vTaskDelay(pdMS_TO_TICKS(200));   /* drain console */
    board_deep_sleep(sleep_s);
}
