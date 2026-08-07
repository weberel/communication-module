/*
 * ecoTrace datalogger
 * ===================
 * Solar-powered remote logger: samples everything every 5 minutes, stores to SPI
 * flash, uploads to ThingsBoard twice a day, deep-sleeps in between.
 *
 * Each wake:
 *   1. read the full battery/charger state (BQ25792) + the on-board I2C sensors
 *      (LTR-303 light, SC7A20 accel),
 *   2. run one solar-management pass (solar.cpp): MPPT step to maximise input
 *      power, per-day harvest accounting, and the weather-adaptive charge target
 *      (good weather -> stop at ~80 % SoC, bad weather -> allow 100 %),
 *   3. append one 64-byte record to the ring log on the 16 MB SPI flash
 *      (flash_log.cpp -- power-loss safe, ~2.5 years of capacity),
 *   4. if an upload is due (every 12 h, or first boot, with retry/backoff), drain
 *      the backlog to ThingsBoard: cellular first, WiFi backup (uplink.cpp),
 *      plus one device-health record. Every upload re-syncs the wall clock
 *      (modem NITZ / SNTP) -- the C6 sleep timer drifts, the 2x/day sync keeps
 *      timestamps honest.
 *   5. deep-sleep until the next sample (everything powered down; the BQ25792
 *      keeps charging autonomously with the settings we left it).
 *
 * Robustness: a task watchdog (WDT_TIMEOUT_S) reboots the board if any wake
 * hangs. RTC RAM survives such resets, so the log cursor and schedule carry on;
 * boot classification below distinguishes a clean timer wake, a crash reboot
 * (state intact -> count it, defer the upload, carry on), and a true cold boot
 * (RTC lost -> full re-init, recover the log from flash).
 *
 * Note on "constant" sampling: the C6's LP core could sample the LP-I2C sensors
 * while the HP core sleeps, but LP-core binaries can't be built in this
 * Arduino/PlatformIO toolchain -- so "constant" here means every wake. The
 * planned ESP-IDF port lifts that limitation.
 *
 * Config in config.h; credentials (SIM_APN, POST_URL, WIFI_*) in secrets.h.
 * Build/flash:  pio run -e datalogger -t upload && pio device monitor
 */
#include <Arduino.h>
#include "esp_random.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "EcoTraceBoard.h"
#include "BQ25792.h"
#include "LTR303.h"
#include "SC7A20.h"
#include "config.h"
#include "record.h"
#include "flash_log.h"
#include "solar.h"
#include "uplink.h"
#include "ota.h"
#include "timeutil.h"

/* ---- state that survives deep sleep (and crash reboots) ---- */
static constexpr uint32_t RTC_STATE_MAGIC = 0x8BADF00D;
static RTC_DATA_ATTR uint32_t s_rtc_magic;         /* valid-state sentinel */
static RTC_DATA_ATTR uint32_t s_uptime_s;          /* approx seconds since cold boot */
static RTC_DATA_ATTR uint32_t s_last_sleep_s;      /* duration of the sleep just ended */
static RTC_DATA_ATTR int32_t  s_next_upload_in_s;  /* countdown to the next upload */
static RTC_DATA_ATTR uint8_t  s_retries_left;
static RTC_DATA_ATTR uint8_t  s_boot_id;
/* health telemetry */
static RTC_DATA_ATTR uint16_t s_boot_count;
static RTC_DATA_ATTR uint16_t s_wake_count;
static RTC_DATA_ATTR uint16_t s_crash_count;       /* consecutive abnormal resets */
static RTC_DATA_ATTR uint16_t s_wdt_trips;
static RTC_DATA_ATTR uint16_t s_upload_fails;      /* consecutive failed attempts */
static RTC_DATA_ATTR uint16_t s_vbat_min_mv;       /* envelope since last upload */
static RTC_DATA_ATTR uint16_t s_vbat_max_mv;
static RTC_DATA_ATTR uint32_t s_awake_ms;          /* awake time since last upload */

BQ25792  bq;
LTR303   light;
SC7A20   accel;
FlashLog flog;

/* Watch the whole wake: if anything hangs, panic-reboot instead of draining the
 * battery at run current. Long stages feed it at checkpoints. */
static void wdtBegin()
{
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms    = (uint32_t)WDT_TIMEOUT_S * 1000;
    cfg.idle_core_mask = 0;
    cfg.trigger_panic = true;
    if (esp_task_wdt_init(&cfg) != ESP_OK)   /* already running (Arduino default) */
        esp_task_wdt_reconfigure(&cfg);
    esp_task_wdt_add(NULL);
}

/* Very crude 1S LiPo SoC from voltage. Good enough for thresholds + trends. */
static uint8_t socFromVoltage(uint16_t vbat_mv)
{
    if (vbat_mv >= 4200) return 100;
    if (vbat_mv <= 3300) return 0;
    return (uint8_t)((vbat_mv - 3300) * 100 / (4200 - 3300));
}

static void readSample(LogRecord& r, const Solar::Status& sol)
{
    memset(&r, 0xFF, sizeof(r));   /* spare bytes stay 0xFF (NOR-friendly) */

    r.ts_s     = clockValid() ? (uint32_t)time(nullptr) : 0;
    r.uptime_s = s_uptime_s;
    r.boot_id  = s_boot_id;

    r.vbat_mv   = bq.readVbat_mV();
    r.ibat_ma   = bq.readIbat_mA();
    r.vbus_mv   = bq.readVbus_mV();
    r.ibus_ma   = bq.readIbus_mA();
    r.vac2_mv   = bq.readVac2_mV();
    r.vsys_mv   = bq.readVsys_mV();
    r.chg_stat  = (uint8_t)bq.chargeState();
    bq.readFaults(r.fault0, r.fault1);
    r.soc_pct   = socFromVoltage(r.vbat_mv);

    r.vindpm_mv   = sol.vindpm_mv;
    r.vreg_mv     = sol.vreg_mv;
    r.harvest_mah = sol.harvest_today_mah;
    r.flags = (sol.solar_present ? RECF_SOLAR   : 0) |
              (sol.usb_present   ? RECF_USB     : 0) |
              (sol.weather_good  ? RECF_WEATHER : 0) |
              (sol.eco_target    ? RECF_ECO_CHG : 0);

    /* On-board I2C sensors (always-on 3V3 rail; low-powered between reads). */
    r.light_ch0 = r.light_ch1 = 0;
    if (light.begin()) {
        delay(120);   /* one ALS integration period after leaving standby */
        uint16_t c0, c1;
        if (light.read(c0, c1)) { r.light_ch0 = c0; r.light_ch1 = c1; }
        light.powerDown();
    }
    r.acc_mg[0] = r.acc_mg[1] = r.acc_mg[2] = 0;
    if (accel.begin()) {
        delay(20);    /* first sample at 100 Hz */
        int16_t ax, ay, az;
        if (accel.readMilliG(ax, ay, az)) {
            r.acc_mg[0] = ax; r.acc_mg[1] = ay; r.acc_mg[2] = az;
        }
        accel.powerDown();
    }
}

/* QON button (GPIO2, shared with the BQ25792's QON input; external pull-up,
 * button pulls low). Short press while asleep = wake for an immediate sample.
 * BUTTON_OTA_PRESSES presses in a row = WiFi OTA mode (testing convenience).
 * Held BUTTON_SHIP_HOLD_MS = power off via BQ ship mode (BATFET opens,
 * ~129 uA); wake by holding the button ~1 s (BQ tSM_EXIT). A ~10 s hold
 * triggers the BQ's own hardware power cycle regardless of firmware. */
static void handleButton(void)
{
    pinMode(ECO_PIN_QON, INPUT);
    if (!EcoTrace::wokeFromButton() && digitalRead(ECO_PIN_QON) == HIGH)
        return;
    uint8_t presses = 1;   /* the press that woke us (it may already be released) */

    /* If that press is still held, watch for the ship-mode hold. */
    uint32_t t0 = millis();
    while (digitalRead(ECO_PIN_QON) == LOW) {
        if (millis() - t0 >= BUTTON_SHIP_HOLD_MS) {
            EcoTrace::ledOn();   /* feedback: power-off armed */
            Serial.println("button held: entering ship mode (hold ~1 s to wake)");
            Serial.flush();
            bq.enterShipMode();
            delay(1000);
            /* Still running means an adapter is holding VSYS up. The BATFET is
             * already open (note: battery does NOT charge in ship mode), so the
             * board goes dark the moment the cable is pulled. Park cheaply. */
            Serial.println("adapter present - board powers off once unplugged");
            EcoTrace::ledOff();
            EcoTrace::deepSleepSeconds(UPLOAD_PERIOD_S);
        }
        delay(10);
    }

    /* Count further presses; each one restarts the window. The boot after the
     * wake press takes a few hundred ms, so "press 3x" in a normal rhythm lands
     * presses 2 and 3 in here. */
    uint32_t window_end = millis() + BUTTON_MULTIPRESS_WINDOW_MS;
    while ((int32_t)(window_end - millis()) > 0 && presses < BUTTON_OTA_PRESSES) {
        if (digitalRead(ECO_PIN_QON) == LOW) {
            delay(30);                                    /* debounce */
            if (digitalRead(ECO_PIN_QON) == LOW) {
                presses++;
                EcoTrace::ledOn(); delay(30); EcoTrace::ledOff();   /* press feedback */
                while (digitalRead(ECO_PIN_QON) == LOW) delay(10);  /* wait for release */
                window_end = millis() + BUTTON_MULTIPRESS_WINDOW_MS;
            }
        }
        delay(5);
    }

    if (presses >= BUTTON_OTA_PRESSES) {
        Serial.printf("button: %u presses - WiFi OTA mode\n", presses);
        Ota::runWindow();   /* reboots on a successful push, else returns */
    } else {
        Serial.printf("button: %u press(es) - extra sample\n", presses);
    }
}

void setup()
{
    Serial.begin(115200);
    wdtBegin();

    /* Boot classification: clean timer wake / crash reboot with RTC state intact /
     * true cold boot (power-on, RTC RAM lost). */
    bool timer_wake  = EcoTrace::wokeFromTimer();
    bool button_wake = EcoTrace::wokeFromButton();
    bool cold        = (s_rtc_magic != RTC_STATE_MAGIC);
    bool crashed     = !cold && !timer_wake && !button_wake;
    esp_reset_reason_t why = esp_reset_reason();

    if (cold) delay(1500);   /* give USB CDC time to enumerate on the bench only */

    EcoTrace::beginBoard();
    EcoTrace::beginI2C();
    EcoTrace::beginSPI();

    handleButton();

    if (cold) {
        s_uptime_s         = 0;
        s_last_sleep_s     = 0;
        s_next_upload_in_s = 0;          /* upload (and clock-sync) on first boot */
        s_retries_left     = UPLOAD_MAX_RETRIES;
        s_boot_id          = (uint8_t)esp_random();
        s_boot_count       = 0;
        s_wake_count       = 0;
        s_crash_count      = 0;
        s_wdt_trips        = 0;
        s_upload_fails     = 0;
        s_vbat_min_mv      = 0xFFFF;
        s_vbat_max_mv      = 0;
        s_awake_ms         = 0;
        Solar::reset();
        s_rtc_magic = RTC_STATE_MAGIC;
        Serial.printf("cold boot (id %u, reset reason %d)\n", s_boot_id, (int)why);
    } else if (crashed) {
        /* The previous wake died (watchdog, panic, brownout, manual reset). Keep
         * all state, note it, and don't retry the likely culprit immediately. */
        s_crash_count++;
        if (why == ESP_RST_TASK_WDT || why == ESP_RST_INT_WDT ||
            why == ESP_RST_WDT || why == ESP_RST_PANIC)
            s_wdt_trips++;
        if (s_next_upload_in_s <= 0) s_next_upload_in_s = UPLOAD_RETRY_S;
        Serial.printf("recovered from abnormal reset (reason %d, %u in a row)\n",
                      (int)why, s_crash_count);
    } else {
        s_wake_count++;
        if (timer_wake) {
            /* A button wake cuts the sleep short by an unknown amount, so leave
             * the uptime/upload bookkeeping to the timer wakes only. */
            s_uptime_s         += s_last_sleep_s;
            s_next_upload_in_s -= (int32_t)s_last_sleep_s;
        }
    }
    s_boot_count++;

    /* After a crash the log cursors in RTC RAM may predate a half-finished
     * append -- re-recover from the chip in that case (cheap). */
    if (!flog.begin(cold || crashed))
        Serial.println("WARNING: SPI flash not found -- samples will be lost!");

    /* Charger up: ADC on, profile applied (VREG is set by the solar manager). */
    Solar::Status sol = {};
    bool bq_ok = bq.begin();
    if (bq_ok) {
        bq.enableADC();
        bq.enableIbatSensing();
        bq.configureCharging(CHARGE_CURRENT_MA, 0 /* set per source below */, 0);
        bq.enableACDRV1(true);   /* USB path gate is off at POR */
        bq.enableACDRV2(true);   /* solar path gate */
        sol = Solar::onWake(bq, SAMPLE_INTERVAL_S, localDayNum());
    } else {
        Serial.println("WARNING: BQ25792 not found -- battery data will be zero.");
    }
    esp_task_wdt_reset();

    /* Sample -> flash. */
    LogRecord r;
    readSample(r, sol);
    flog.append(r);
    if (r.vbat_mv) {
        if (r.vbat_mv < s_vbat_min_mv) s_vbat_min_mv = r.vbat_mv;
        if (r.vbat_mv > s_vbat_max_mv) s_vbat_max_mv = r.vbat_mv;
    }
    Serial.printf("seq %lu: VBAT=%umV IBAT=%+dmA SOC=%u%% | in: %s VINDPM=%umV "
                  "IBUS=%dmA | harvest %umAh (prev %u) weather=%s target=%umV | "
                  "lux0=%u acc=[%d,%d,%d] | pending %lu\n",
                  (unsigned long)r.seq, r.vbat_mv, r.ibat_ma, r.soc_pct,
                  sol.usb_present ? "USB" : sol.solar_present ? "solar" : "none",
                  r.vindpm_mv, r.ibus_ma,
                  sol.harvest_today_mah, sol.harvest_prev_mah,
                  sol.weather_good ? "good" : "bad", r.vreg_mv,
                  r.light_ch0, r.acc_mg[0], r.acc_mg[1], r.acc_mg[2],
                  (unsigned long)flog.pendingCount());
    esp_task_wdt_reset();

    /* Upload if due -- and only if the battery can afford the modem burst. */
    if (s_next_upload_in_s <= 0) {
        bool can_afford = r.vbat_mv >= UPLOAD_MIN_VBAT_MV ||
                          sol.usb_present || sol.solar_present;
        if (!can_afford) {
            Serial.println("upload due but battery low -- deferring");
            s_next_upload_in_s = UPLOAD_RETRY_S;
        } else {
            Uplink::StatusInfo si = {};
            si.boot_id      = s_boot_id;
            si.reset_reason = (uint8_t)why;
            si.boot_count   = s_boot_count;
            si.wake_count   = s_wake_count;
            si.crash_count  = s_crash_count;
            si.wdt_trips    = s_wdt_trips;
            si.upload_fails = s_upload_fails;
            si.vbat_min_mv  = (s_vbat_min_mv == 0xFFFF) ? 0 : s_vbat_min_mv;
            si.vbat_max_mv  = s_vbat_max_mv;
            si.awake_ms     = s_awake_ms + millis();
            si.uptime_s     = s_uptime_s;

            Uplink::Result u = Uplink::uploadAll(flog, SAMPLE_INTERVAL_S, si);
            if (u.all_sent) {
                s_next_upload_in_s = UPLOAD_PERIOD_S;
                s_retries_left     = UPLOAD_MAX_RETRIES;
                s_upload_fails     = 0;
                s_vbat_min_mv = s_vbat_max_mv = r.vbat_mv ? r.vbat_mv : s_vbat_max_mv;
                if (!r.vbat_mv) s_vbat_min_mv = 0xFFFF;
                s_awake_ms = 0;
            } else {
                s_upload_fails++;
                if (s_retries_left > 0) {
                    s_retries_left--;
                    s_next_upload_in_s = UPLOAD_RETRY_S;
                } else {
                    s_next_upload_in_s = UPLOAD_PERIOD_S;   /* give up until next slot */
                    s_retries_left     = UPLOAD_MAX_RETRIES;
                }
            }
        }
    }

    /* Everything down, then sleep. Charging continues autonomously. */
    if (bq_ok) bq.disableADC();
    flog.sleep();

    uint32_t sleep_s = SAMPLE_INTERVAL_S;
    if (r.vbat_mv && r.vbat_mv < CRITICAL_VBAT_MV && !sol.usb_present && !sol.solar_present)
        sleep_s = SAMPLE_INTERVAL_S * CRITICAL_INTERVAL_MULT;
    if (s_crash_count >= CRASH_SLOWDOWN_COUNT)
        sleep_s = SAMPLE_INTERVAL_S * CRITICAL_INTERVAL_MULT;   /* containment */

    /* Reaching this point = the wake completed; the crash streak is over. */
    s_crash_count  = 0;
    s_awake_ms    += millis();
    s_uptime_s    += millis() / 1000;   /* count awake time into uptime too */
    s_last_sleep_s = sleep_s;

    Serial.printf("sleeping %lus (next upload in %lds)\n",
                  (unsigned long)sleep_s, (long)s_next_upload_in_s);
    Serial.flush();
    EcoTrace::deepSleepSeconds(sleep_s);   /* never returns */
}

void loop() {}
