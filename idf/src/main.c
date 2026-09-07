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
#include <math.h>
#include <string.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_pm.h"
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
#include "uss_link.h"   /* USS_ST_BOOT: the module resets its own counters */
#include "i2c_bus.h"
#include "esp_timer.h"
#include "wf280a.h"
#include "uplink.h"

static const char *TAG = "ecotrace";

/* Set for the duration of a wake that the accelerometer triggered. */
static bool s_motion_flag;

/* When the switched SENSOR rail came up this wake; the ultrasonic module needs
 * BOARD_SENSOR_BOOT_MS from that point before its I2C slave answers. */
static int64_t s_sensor_up_us;

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
/* Previous capture counters, for the per-record delta. Free-running u16s on the
 * module, so the subtraction is correct modulo 65536 and needs no handshake. */
static RTC_DATA_ATTR uint16_t s_prev_cap_n;
static RTC_DATA_ATTR uint16_t s_prev_cap_badcode;
static RTC_DATA_ATTR uint16_t s_prev_cap_badsnr;
static RTC_DATA_ATTR uint8_t  s_cap_seen;   /* a previous sample exists */
/* Set for the wake that a button press caused. A press means a person is at the
 * board asking for attention, so it overrides the USS recovery backoff: the
 * backoff exists to stop a node with no gas cell power-cycling its rail every
 * 5 minutes forever, but it also means a module hot-plugged onto a live bus --
 * which can leave it wedged half-powered off the bus's ESD clamps -- waits up
 * to an hour for the rail cycle that would clear it. Observed 2026-09-06. */
static bool s_force_uss_recovery;
/* upload scheduling: bounded tempo (2026-08-11 spec) */
static RTC_DATA_ATTR int32_t  s_next_upload_in_s;
static RTC_DATA_ATTR uint8_t  s_retries_left;
static RTC_DATA_ATTR uint32_t s_last_deep_uptime;   /* deep search once/day */
static RTC_DATA_ATTR uint8_t  s_upload_inflight;
static RTC_DATA_ATTR uint8_t  s_upload_crashed;
/* Consecutive wakes on which the ultrasonic module did not answer. Used to back
 * the recovery off: retrying a rail power-cycle every 5 minutes forever is the
 * worst case for power on a node whose module is absent or dead. */
static RTC_DATA_ATTR uint16_t s_uss_fail_streak;
/* Consecutive out-of-turn motion wakes, reset by any ordinary timer wake. */
static RTC_DATA_ATTR uint8_t  s_motion_streak;

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
               (sol->eco_target    ? RECF_ECO_CHG : 0) |
               (s_motion_flag      ? RECF_MOTION  : 0);

    /* Log what is actually on the bus. Cheap (absent devices NACK immediately)
     * and it turns "the reading is zero" into "that chip is not there", which
     * are entirely different faults. */
    eco_i2c_scan();

    r->sensor_ok = bq_ok ? SOK_BQ : 0x00;


    /* Sampled every wake. Duty-cycling it was considered and rejected: the
     * 150 ms integration wait is only ~0.5 mAh/day (0.4 % of the budget), and
     * leaving the sensor in continuous mode instead would cost ~2.4 mAh/day of
     * its own active current -- five times more than it saves. */
    uint16_t c0, c1;
    if (ltr303_sample(&c0, &c1)) {
        r->light_ch0 = c0; r->light_ch1 = c1;
        r->sensor_ok |= SOK_LTR303;
    } else { r->light_ch0 = r->light_ch1 = 0; }

    int16_t ax, ay, az;
    if (sc7a20_sample(&ax, &ay, &az)) {
        r->acc_mg[0] = ax; r->acc_mg[1] = ay; r->acc_mg[2] = az;
        r->sensor_ok |= SOK_SC7A20;
    } else {
        r->acc_mg[0] = r->acc_mg[1] = r->acc_mg[2] = 0;
    }

    float mbar, degc;
    if (ms5837_sample(&mbar, &degc)) {
        r->press_dmbar = (uint16_t)(mbar * 10.0f + 0.5f);
        r->temp_cC     = (int16_t)(degc * 100.0f + (degc >= 0 ? 0.5f : -0.5f));
        r->sensor_ok |= SOK_MS5837;
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
        if (elapsed_ms < BOARD_SENSOR_BOOT_MS)
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
    if (s_uss_fail_streak < USS_RETRY_STREAK ||
        (s_wake_count % USS_RETRY_EVERY_N_WAKES) == 0)
        (void) uss_start_auto(USS_AUTO_PERIOD_S);

    bool uss_ok = uss_sample(&u);
    /* Back off the recovery. The power-cycle costs ~2 s of awake time (800 ms
     * rail-down plus the module's boot and a second sample), which is the
     * single most expensive thing a wake can do. Worth it when a working module
     * has wedged; pure waste every 5 minutes on a node whose module is absent,
     * unpowered, or dead -- which is exactly the state a bench board or a
     * fleet unit without a gas cell sits in. After USS_RETRY_STREAK failures,
     * attempt it only once an hour; a module that comes back is picked up
     * within that hour and costs nothing in between. */
    bool try_recovery = uss_ok ? false
                      : (s_uss_fail_streak < USS_RETRY_STREAK) ||
                        ((s_wake_count % USS_RETRY_EVERY_N_WAKES) == 0) ||
                        s_force_uss_recovery;
    if (!uss_ok && !try_recovery) {
        ESP_LOGW(TAG, "USS silent (%u wakes) -- recovery backed off",
                 s_uss_fail_streak);
    }
    if (!uss_ok && try_recovery) {
        ESP_LOGW(TAG, "USS silent, power-cycling the sensor rail");
        /* 800 ms, not 300: the SENSOR rail is a high-side switch with no
         * bleed resistor, so when it opens the node is left floating and the
         * decoupling caps discharge only through the slave's own quiescent
         * draw. Confirmed on the bench 2026-08-15 with an LED across the
         * header -- it fades rather than switching off. Too short a window
         * means no power-on reset at all, which defeats the purpose. */
        board_sensor_power_cycle(800);
        /* The module has just rebooted, so STATUS.AUTO is clear -- restart
         * autonomous mode before sampling, or it would stay in one-shot until
         * the next wake and the totalizer would never accumulate. */
        (void) uss_start_auto(USS_AUTO_PERIOD_S);
        uss_ok = uss_sample(&u);
        ESP_LOGW(TAG, "USS after power cycle: %s", uss_ok ? "recovered" : "still silent");
    }
    s_uss_fail_streak = uss_ok ? 0
                      : (s_uss_fail_streak < 0xFFFF ? s_uss_fail_streak + 1 : 0xFFFF);
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
        r->uss_vol_ml    = u.vol_ml;
        /* Delta since the previous sample. Skipped on the first sample after a
         * cold boot and whenever the module reports the BOOT bit -- its
         * counters restart at zero on its own reset, so a difference across
         * that boundary is meaningless rather than merely inaccurate. */
        uint16_t dn  = (uint16_t)(u.cap_n       - s_prev_cap_n);
        uint16_t dbc = (uint16_t)(u.cap_badcode - s_prev_cap_badcode);
        uint16_t dbs = (uint16_t)(u.cap_badsnr  - s_prev_cap_badsnr);
        /* Sanity-check the delta instead of trusting the BOOT bit alone.
         *
         * BOOT is cleared by the first command, and uss_start_auto() runs
         * before uss_sample() -- so by the time we look, a module that rebooted
         * this wake already reports BOOT clear and its counters have restarted
         * at zero. The subtraction then wraps and produces nonsense: on
         * 2026-09-06 16:07 that published n=64582, badsnr=65533, i.e. "101.5 %
         * bad", sitting next to three honest records reading 0.0-0.3 %.
         *
         * Two invariants catch it without needing to know why. Bad counts can
         * never exceed the total, and the module cannot have run more captures
         * than the interval allows -- 4x the nominal rate is generous headroom
         * for a period change and still rejects a wrap by three orders. */
        bool plausible = (dbc <= dn) && (dbs <= dn) &&
                         (dn <= (uint16_t)(SAMPLE_INTERVAL_S * 4u));
        if (s_cap_seen && !(u.status & USS_ST_BOOT) && plausible) {
            r->uss_cap_n       = dn;
            r->uss_cap_badcode = dbc;
            r->uss_cap_badsnr  = dbs;
        } else {
            r->uss_cap_n = r->uss_cap_badcode = r->uss_cap_badsnr = 0;
        }
        s_prev_cap_n       = u.cap_n;
        s_prev_cap_badcode = u.cap_badcode;
        s_prev_cap_badsnr  = u.cap_badsnr;
        s_cap_seen         = 1;
        r->uss_tof_ups_q40 = u.tof_ups_q40;
        r->uss_tof_dns_q40 = u.tof_dns_q40;
        r->sensor_ok |= SOK_USS;
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
        r->uss_vol_ml = 0;
        r->uss_tof_ups_q40 = r->uss_tof_dns_q40 = 0;
    }

    uint32_t praw, traw;
    uint8_t  wst;
    if (wf280a_sample(&praw, &traw, &wst)) {
        r->wf_praw = praw;
        r->wf_traw = traw;
        r->sensor_ok |= SOK_WF280A;
        ESP_LOGI(TAG, "WF280A: praw=%lu traw=%lu st=0x%02X",
                 (unsigned long)praw, (unsigned long)traw, wst);
    } else {
        r->wf_praw = r->wf_traw = 0;
    }
}

/* ===================== power audit ==========================================
 *
 * A direct, automatic measurement of what the ultrasonic board's analog front
 * end costs, in microamps, replacing the multi-day battery-discharge slopes
 * that could not resolve anything smaller than about 10 % of the budget.
 *
 * Method: average the BQ25792's IBAT with the module's rails up, command
 * AUTO_STOP (which drops its 5 V boost and both AFE rails but leaves the
 * MSP430 running), average again, then restore. The difference is the rails.
 *
 * Two design constraints worth recording, because both were nearly got wrong:
 *
 *  - The load step is NOT "cut the SENSOR rail". Making that rail actually fall
 *    needs eco_i2c_hold_low(), because the module's ESD clamps back-feed from an
 *    idle-high bus -- and that call deletes the I2C bus the ammeter itself sits
 *    on. The meter would go blind exactly when the load went away.
 *
 *  - Both halves are taken seconds apart in the same CPU state, so every slow
 *    drift (temperature, pack voltage, sun) cancels. That is what a differential
 *    buys over comparing absolute numbers taken days apart.
 *
 * Only valid on battery: with a charger attached the load step is absorbed by
 * the input and IBAT barely moves. Costs one interval of totalized volume,
 * since AUTO_START zeroes it -- so this runs on a button press, not every wake.
 */
#define AUDIT_CYCLES       3    /* interleaved up/down pairs */
#define AUDIT_SAMPLES    160    /* x 50 ms = 8 s per half */
#define AUDIT_SPACING_MS  50    /* > one ADC conversion cycle, or we average
                                 * the same register contents 160 times */
#define AUDIT_SETTLE_MS  700

static int32_t s_audit_rail_ua;   /* AFE rails + boost, uA (mean of the pairs) */
static int32_t s_audit_se_ua;     /* standard error of that mean */
static int32_t s_audit_base_ua;   /* node discharge with those rails down, uA */
static bool    s_audit_valid;

static void power_audit(bool uss_present)
{
    s_audit_valid = false;
    if (!uss_present) return;
    if (bq_vbus_present()) {
        ESP_LOGI(TAG, "power audit skipped: charger attached");
        return;
    }
    /* Establish the precondition rather than assuming it. The recovery backoff
     * can leave AUTO un-asserted for a wake or two after the module reappears,
     * and uss_stop_auto() succeeds trivially when it was never running -- so
     * both halves would be measured with the rails already down and the audit
     * would report a confident, entirely wrong 0 uA. */
    /* Force a restart so the configured period is actually applied.
     * uss_start_auto() deliberately returns early when STATUS.AUTO is already
     * set -- re-issuing AUTO_START would zero the totalizer -- which also means
     * it never writes AUTO_PERIOD to a module that is already running. Without
     * this stop first, changing USS_AUTO_PERIOD_S would have no effect on a
     * module that never rebooted, and the experiment would silently measure the
     * old rate. */
    (void) uss_stop_auto();
    if (!uss_start_auto(USS_AUTO_PERIOD_S)) {
        ESP_LOGW(TAG, "power audit: autonomous mode unavailable, skipping");
        return;
    }

    /* Pin the CPU out of light sleep for the whole audit. Otherwise the ESP's
     * own draw depends on how tickless idle happened to schedule around our
     * I2C reads, which is a variable we do not want inside a differential. */
    esp_pm_lock_handle_t lock = NULL;
    if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "audit", &lock) != ESP_OK)
        lock = NULL;
    if (lock) esp_pm_lock_acquire(lock);

    /* Interleaved up/down pairs, not one long A followed by one long B.
     *
     * The first attempt (idf-0.12, 2026-09-06) used 40 samples over 2 s per
     * half and returned +7.5 mA then -1.9 mA on the same hardware minutes
     * apart -- the second physically impossible, reading rails-down as drawing
     * MORE than rails-up. All of the scatter was in the rails-up half, because
     * the module fires a capture burst once a second and 2 s of instantaneous
     * samples either lands on those bursts or does not. Eight seconds covers
     * ~8 bursts per half, interleaving cancels any slow drift between halves,
     * and the spread across pairs is published so the number arrives with its
     * own error bar instead of an assumption of precision. */
    int32_t diff[AUDIT_CYCLES];
    int32_t last_down = 0;
    for (int c = 0; c < AUDIT_CYCLES; c++) {
        esp_task_wdt_reset();
        (void) uss_start_auto(USS_AUTO_PERIOD_S);
        vTaskDelay(pdMS_TO_TICKS(AUDIT_SETTLE_MS));
        int32_t up = bq_ibat_avg_ua(AUDIT_SAMPLES, AUDIT_SPACING_MS);

        esp_task_wdt_reset();
        if (!uss_stop_auto()) {
            ESP_LOGW(TAG, "power audit: AUTO_STOP refused mid-run, aborting");
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(AUDIT_SETTLE_MS));
        int32_t down = bq_ibat_avg_ua(AUDIT_SAMPLES, AUDIT_SPACING_MS);

        /* IBAT is negative while discharging, so removing a load makes it less
         * negative and the difference comes out positive. */
        diff[c] = down - up;
        last_down = down;
        ESP_LOGI(TAG, "audit pair %d/%d: up %ld uA, down %ld uA -> %ld uA",
                 c + 1, AUDIT_CYCLES, (long)up, (long)down, (long)diff[c]);
    }

    {
        double mean = 0.0;
        for (int i = 0; i < AUDIT_CYCLES; i++) mean += diff[i];
        mean /= AUDIT_CYCLES;
        double var = 0.0;
        for (int i = 0; i < AUDIT_CYCLES; i++) {
            double e = diff[i] - mean;
            var += e * e;
        }
        var /= (AUDIT_CYCLES - 1);                 /* sample variance */
        double se = sqrt(var / AUDIT_CYCLES);      /* of the mean */

        s_audit_rail_ua = (int32_t) mean;
        s_audit_se_ua   = (int32_t) se;
        s_audit_base_ua = last_down;
        s_audit_valid   = true;
        ESP_LOGI(TAG, "power audit: USS AFE = %ld +/- %ld uA (n=%d)",
                 (long)s_audit_rail_ua, (long)s_audit_se_ua, AUDIT_CYCLES);
    }

done:
    (void) uss_start_auto(USS_AUTO_PERIOD_S);      /* always restore */
    if (lock) { esp_pm_lock_release(lock); esp_pm_lock_delete(lock); }
}

void app_main(void)
{
    esp_reset_reason_t why = esp_reset_reason();
    bool timer_wake  = board_woke_from_timer();
    bool button_wake = board_woke_from_button();
    bool motion_wake = board_woke_from_motion();
    bool cold        = (s_rtc_magic != RTC_STATE_MAGIC);
    bool crashed     = !cold && !timer_wake && !button_wake && !motion_wake;

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
    /* Only stamp a fresh bring-up time when the rail was ACTUALLY off, i.e. on
     * a cold boot. board_deep_sleep() deliberately holds this rail high through
     * sleep, so on an ordinary timer wake the ultrasonic module never rebooted
     * and there is nothing to wait for -- yet the wait below was being paid on
     * every wake: 3 s of awake time, 288 times a day, ~9.6 mAh/day, about 8 %
     * of the whole node budget, spent waiting for a boot that did not happen.
     * (The genuine reboot path, board_sensor_power_cycle(), already does its
     * own BOARD_SENSOR_BOOT_MS wait internally.) */
    s_sensor_up_us = (cold || crashed)
                   ? esp_timer_get_time()          /* rail really was off/glitched */
                   : esp_timer_get_time() - (int64_t)BOARD_SENSOR_BOOT_MS * 1000;

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
    s_motion_flag = motion_wake;
    if (motion_wake) {
        if (s_motion_streak < 0xFF) s_motion_streak++;
        ESP_LOGI(TAG, "motion wake (%u in a row)", s_motion_streak);
    } else if (timer_wake || button_wake) {
        s_motion_streak = 0;
    }

    s_boot_count++;
    s_force_uss_recovery = button_wake;   /* consumed by read_sample() below */

    /* Charger first: park-mode decision needs VBAT before we touch anything. */
    bool bq_ok = bq_begin();
    if (bq_ok) {
        /* Order matters: EN_IBAT/SFET_PRESENT before the ADC starts. Enabling
         * the sense path after bq_adc_enable()'s 150 ms settle left the IBAT
         * channel un-converted when read_sample() ran a moment later -- the
         * discharge column read a flat zero for 22 days. */
        bq_ibat_sense(true);
        bq_adc_enable(true);
    }
    uint16_t vbat = bq_ok ? bq_vbat_mv() : 0;

    /* Battery park mode: below PARK_VBAT_MV (and nothing plugged in) do the
     * absolute minimum -- no flash writes, no sensors, long sleeps -- until the
     * pack recovers past the hysteresis threshold. Brownout mid-flash-write is
     * the failure this avoids. */
    bool usb_present = bq_ok && bq_ac1_present();

    /* Power management. Frequency scaling always; automatic LIGHT SLEEP only
     * when USB is absent.
     *
     * Light sleep stops the clock during every vTaskDelay -- sensor
     * conversions, the ADC settle, the rail cycle -- which is most of a wake,
     * so it is the single biggest saving available on this side. But the C6's
     * USB-Serial/JTAG peripheral loses its clock with it, and the console does
     * not take a PM lock, so the port stops enumerating: no logs, and no
     * wake-window flashing. On a deployed node nobody is watching and that is
     * a fine trade; on the bench it makes the board unreachable.
     *
     * USB presence is exactly the right discriminator -- if a cable is plugged
     * in, someone is working on it. */
    esp_pm_config_t pm = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,                    /* XTAL */
        .light_sleep_enable = !usb_present,
    };
    esp_err_t pm_err = esp_pm_configure(&pm);
    if (pm_err != ESP_OK)
        ESP_LOGW(TAG, "esp_pm_configure failed: %s", esp_err_to_name(pm_err));
    else
        ESP_LOGI(TAG, "PM: 40-80 MHz, light sleep %s",
                 usb_present ? "OFF (USB attached)" : "on");

    bool input_present = bq_ok && (usb_present || bq_ac2_present());
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
        bq_configure_charging(CHARGE_CURRENT_MA, 0, 0);
        bq_enable_acdrv1(true);
        bq_enable_acdrv2(true);
        /* Local calendar day for harvest/sun rollover once the clock is valid;
         * falls back to a 24 h counter before the first sync. */
        time_t now = time(NULL);
        int32_t day_num = (now >= 1767225600 && now < 2082758400)
                        ? (int32_t)((now + TZ_OFFSET_MIN * 60) / 86400) : -1;
        /* Cheap bookkeeping every wake; the expensive hill climb every Nth. */
        bool do_mppt = (s_wake_count % MPPT_EVERY_N_WAKES) == 0;
        sol = solar_on_wake(SAMPLE_INTERVAL_S, day_num, do_mppt);
    } else {
        ESP_LOGE(TAG, "BQ25792 not found -- battery data will be zero");
    }

    LogRecord r;
    read_sample(&r, &sol, bq_ok);
    flashlog_append(&r);
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
             (r.sensor_ok & SOK_BQ) ? 'y' : 'N',
             (r.sensor_ok & SOK_LTR303) ? 'y' : 'N',
             (r.sensor_ok & 0x04) ? 'y' : 'N', (r.sensor_ok & 0x08) ? 'y' : 'N',
             (r.sensor_ok & 0x10) ? 'y' : 'N', (r.sensor_ok & 0x20) ? 'y' : 'N');

    /* A button press means someone is standing at the board wanting an answer,
     * which is exactly when spending ~5 s and one totalizer interval on a real
     * current measurement is worth it. The same press forces an upload below,
     * so the number reaches ThingsBoard in the same wake. */
    if (button_wake) power_audit((r.sensor_ok & SOK_USS) != 0);

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
            .audit_valid  = s_audit_valid,
            .audit_rail_ua = s_audit_rail_ua,
            .audit_se_ua  = s_audit_se_ua,
            .audit_base_ua = s_audit_base_ua,
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

    /* Clear the latch (or the INT line stays asserted and the next sleep returns
     * immediately) and re-arm -- unless we are in a motion burst, in which case
     * leave it disarmed until the next ordinary timer wake. */
    (void) sc7a20_motion_fired();
    bool arm_motion = (s_motion_streak < MOTION_WAKE_BURST_MAX);
    if (arm_motion) arm_motion = sc7a20_arm_motion(MOTION_THRESHOLD_MG);
    else ESP_LOGW(TAG, "motion wakes suppressed (%u in a row)", s_motion_streak);
    board_set_motion_wake(arm_motion);

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
    /* Draining the console costs 200 ms of awake time on EVERY wake -- ~0.6
     * mAh/day at a 5 min interval, for output nobody can see unless USB is
     * attached. Pay it only when someone is actually watching. */
    if (sol.usb_present) vTaskDelay(pdMS_TO_TICKS(200));
    board_deep_sleep(sleep_s);
}
