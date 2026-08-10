/*
 * datalogger/config.h  --  tunables for the ecoTrace datalogger.
 *
 * Duty cycle, charge profile, MPPT, weather thresholds, upload schedule.
 * Credentials (SIM_APN, POST_URL, WIFI_SSID/WIFI_PASS) live in secrets.h.
 */
#pragma once

/* ---- Duty cycle --------------------------------------------------------------
 * The board wakes every SAMPLE_INTERVAL_S, samples everything (sensors + full
 * battery/charger state), appends one record to the SPI-flash ring log, runs one
 * solar-management pass, and deep-sleeps. Below CRITICAL_VBAT_MV the interval is
 * stretched to save the last of the battery (logging continues, slower). */
#define FW_VERSION              "dl-2.14"  /* printed at boot + sent with telemetry;
                                            * bump before an OTA push to see it took */

#define SAMPLE_INTERVAL_S       300     /* 5 min */
#define CRITICAL_VBAT_MV        3350
#define CRITICAL_INTERVAL_MULT  6       /* -> 30 min when critical */

/* ---- Upload ------------------------------------------------------------------
 * Twice a day the backlog is drained to ThingsBoard: cellular first, WiFi as
 * backup. On total failure it retries after UPLOAD_RETRY_S, up to
 * UPLOAD_MAX_RETRIES times, then waits for the next regular slot. Uploads are
 * skipped (data keeps accumulating in flash) while the battery is below
 * UPLOAD_MIN_VBAT_MV and nothing is plugged in -- a modem burst is the single
 * most expensive thing this board does. */
#define UPLOAD_PERIOD_S         (12 * 3600)   /* twice a day */
#define UPLOAD_RETRY_S          (30 * 60)
#define UPLOAD_MAX_RETRIES      3
#define UPLOAD_MIN_VBAT_MV      3500
#define UPLOAD_BATCH_RECORDS    8       /* records per HTTP POST (~3.5 KB JSON) */
#define UPLOAD_BATCH_GAP_MS     400     /* pause between POSTs -- ThingsBoard Cloud
                                         * intermittently 500s back-to-back bursts */
#define NTP_SERVER              "pool.ntp.org"
#define TZ_OFFSET_MIN           120     /* local time for the harvest day-rollover (CEST) */

/* ---- Charge profile (BQ25792, 1S LiPo) --------------------------------------
 * VREG_FULL is used in bad weather to build maximum reserve; VREG_ECO (~80 % SoC)
 * whenever the weather is good, to keep the cell off 4.2 V and age it slower. */
#define VREG_FULL_MV            4200
#define VREG_ECO_MV             4050    /* ~80 % SoC resting */
#define CHARGE_CURRENT_MA       1000    /* ICHG */
#define INPUT_LIMIT_USB_MA      2000    /* IINDPM on USB */
#define INPUT_LIMIT_SOLAR_MA    3000    /* IINDPM on solar: above any panel we'd fit,
                                         * so VINDPM (MPPT) is the only active loop */
#define USB_VINDPM_MV           4400    /* park VINDPM here on USB power */

/* ---- Solar MPPT (VINDPM fractional-Voc + perturb & observe) ------------------
 * Every MPPT_VOC_PERIOD_WAKES wakes (or when input current collapses below
 * MPPT_COLLAPSE_MA) the panel's open-circuit voltage is measured via a brief HIZ
 * and VINDPM jumps to MPPT_FOC_PCT of it; in between, MPPT_STEPS_PER_WAKE small
 * P&O steps track the max-power point. Bounds are wide -- set them around your
 * panel's Voc range if you know it. */
#define MPPT_FOC_PCT            80      /* VINDPM = 80 % of Voc (crystalline Si sweet spot) */
#define MPPT_VOC_PERIOD_WAKES   12      /* re-anchor every ~1 h */
#define MPPT_VOC_SETTLE_MS      250
#define MPPT_COLLAPSE_MA        30
#define MPPT_STEPS_PER_WAKE     3
#define MPPT_STEP_MV            100
#define MPPT_SETTLE_MS          150
#define MPPT_VINDPM_START_MV    14000   /* used only until the first Voc measurement */
#define MPPT_VINDPM_MIN_MV      5000
#define MPPT_VINDPM_MAX_MV      22000

/* ---- Robustness --------------------------------------------------------------
 * Task watchdog: if a wake ever hangs (I2C lockup, modem AT wedge, WiFi stall)
 * the WDT panics and reboots instead of draining the pack at run current. Long
 * stages feed it at checkpoints. After an abnormal reset the next upload is
 * deferred by UPLOAD_RETRY_S (RTC state survives the reboot), and after
 * CRASH_SLOWDOWN_COUNT consecutive crashes the sample interval is stretched as
 * containment until a wake completes cleanly. */
#define WDT_TIMEOUT_S           120
#define CRASH_SLOWDOWN_COUNT    5

/* ---- QON button --------------------------------------------------------------
 * The button pulls the shared QON net low (BQ25792 QON + GPIO2). Short press:
 * wakes the logger for an immediate extra sample. BUTTON_OTA_PRESSES presses in
 * a row (each within BUTTON_MULTIPRESS_WINDOW_MS of the last): WiFi OTA mode --
 * the board joins WIFI_SSID and accepts a firmware push (ArduinoOTA) for
 * OTA_WINDOW_S, then resumes logging. Held BUTTON_SHIP_HOLD_MS: firmware enters
 * BQ ship mode - battery disconnected, ~129 uA - and the board is off until the
 * button is held ~1 s again (or an adapter is plugged in). Independent of
 * firmware, a ~10 s hold makes the BQ hardware-power-cycle the whole board
 * (built-in unbrick/reset). */
#define BUTTON_SHIP_HOLD_MS         3000

/* LED on for the whole time the ESP is awake: a working board visibly pulses
 * once per wake, and a board in ship mode stays dark. Costs ~2 mA for the ~2 s
 * of each wake -- negligible on the bench, set to 0 for deployment. */
#define LED_SHOW_AWAKE              1
#define BUTTON_OTA_PRESSES          3
#define BUTTON_MULTIPRESS_WINDOW_MS 2500

/* ---- WiFi OTA (bench/testing convenience, not the deployment story) ----------
 * Push a new image straight from PlatformIO while the board is in OTA mode:
 *     pio run -e datalogger_ota -t upload
 * No rollback in the Arduino build -- if you push a broken image, recover over
 * USB. The ESP-IDF port will own real (cellular, rollback-safe) OTA. */
#define OTA_HOSTNAME            "ecotrace"
#define OTA_WINDOW_S            300     /* stays in OTA mode this long, then sleeps */

/* ---- Weather detection -------------------------------------------------------
 * Harvested solar charge is integrated per local day. If yesterday's or today's
 * harvest reaches this, the weather counts as "good" and charging caps at
 * VREG_ECO_MV. Scale to your panel + battery: default = half of a 2000 mAh pack. */
#define WEATHER_GOOD_MAH        1000
