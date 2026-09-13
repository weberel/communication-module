/*
 * config.h  --  tunables for the ESP-IDF datalogger.
 * Values carried over from the hardware-validated Arduino build
 * (src/datalogger/config.h), plus the field-robustness floors agreed 2026-08-11.
 */
#pragma once

#define FW_VERSION              "idf-0.19"

/* ---- Duty cycle ---- */
#define SAMPLE_INTERVAL_S       300     /* 5 min */
/* Original note, preserved (it documents the 300 s choice):
 *   Measured 2026-09-08: one wake costs 67.1 mC = 0.0186 mAh, so 300 s costs
 *   5.4 mAh/day and 900 s would cost 1.8. uss_vol_ml is TOTALIZED on the USS
 *   side, so a slower log rate would lose temporal resolution, not volume.
 *   Kept at 300 s for resolution; the 15.7 mAh/day sleep floor is 67% of the
 *   budget and independent of this. */
#define CRITICAL_VBAT_MV        3350
#define CRITICAL_INTERVAL_MULT  6       /* -> 30 min when critical */

/* ---- Battery floors (A7672 needs >=3.4 V at the pin under 2 A bursts;
 * RT9080 dropout wants margin for the ESP) ---- */
#define MODEM_MIN_VBAT_MV       3600    /* no cellular attempts below (resting) */
#define WIFI_MIN_VBAT_MV        3450    /* no WiFi attempts below */
#define PARK_VBAT_MV            3300    /* full park: no flash, no sensors */
#define PARK_RESUME_VBAT_MV     3450    /* hysteresis */
#define PARK_SLEEP_S            3600    /* recheck cadence while parked */

/* ---- Charge profile (BQ25792, 1S LiPo) ---- */
#define VREG_FULL_MV            4200
#define VREG_ECO_MV             4050    /* ~80 % SoC resting */
#define CHARGE_CURRENT_MA       1000
#define INPUT_LIMIT_USB_MA      2000
#define INPUT_LIMIT_SOLAR_MA    3000
#define USB_VINDPM_MV           4400

/* ---- Solar MPPT (fractional-Voc + P&O), validated on hardware ---- */
#define MPPT_FOC_PCT            80
/* Per-wake work that does NOT need to happen every 5 minutes. The battery and
 * charger are still read on every wake -- that is the cheap part and it keeps
 * full diagnostic resolution -- but the expensive extras are duty-cycled:
 *   mppt_step()   ~600-900 ms/wake outdoors (3 perturb steps at 150 ms settle,
 *                 plus the hourly 250 ms Voc). The panel's operating point does
 *                 not move on a 5-minute timescale.
 * Voc cadence is now measured in SECONDS (MPPT_VOC_PERIOD_S), so the sun-hours
 * detector stays hourly no matter how the wake interval or these divisors are
 * changed -- it used to be a wake count, which silently broke if either moved. */
#define MPPT_EVERY_N_WAKES      3       /* -> perturb-and-observe every 15 min */
/* Ultrasonic recovery backoff: after this many consecutive silent wakes, stop
 * power-cycling the sensor rail on every wake and try only every Nth. */
/* Motion wake. The accelerometer runs continuously in low-power mode (~2 uA)
 * and pulls the shared INT line on movement, which wakes the ESP out of turn.
 * The burst cap is the safety valve: an installation that vibrates would
 * otherwise wake the node continuously and flatten the battery, so after this
 * many out-of-turn wakes in a row the interrupt is left disarmed until the next
 * ordinary timer wake. */
#define MOTION_THRESHOLD_MG     96      /* rounded to the part's 16 mg/LSB step */
#define MOTION_WAKE_BURST_MAX   5

/* Motion wake. MEASURED 2026-09-08 to cost nothing: the shipping firmware slept
 * at 1467.5 uA with it disabled, against 1467.1 uA with it enabled. It was
 * briefly suspected and cleared. Left ON. */
#define MOTION_WAKE_ENABLE      1

#define USS_RETRY_STREAK        3
#define USS_RETRY_EVERY_N_WAKES 12      /* -> once an hour at a 5 min interval */
#define MPPT_VOC_PERIOD_S       3600    /* open-circuit Voc once an hour */
#define MPPT_VOC_SETTLE_MS      250
#define MPPT_COLLAPSE_MA        30
#define MPPT_STEPS_PER_WAKE     3
#define MPPT_STEP_MV            100
#define MPPT_SETTLE_MS          150
#define MPPT_VINDPM_START_MV    14000
#define MPPT_VINDPM_MIN_MV      5000
#define MPPT_VINDPM_MAX_MV      22000

/* ---- Weather detection ----
 * Two independent signals, either one declares the day "good":
 *  - harvest (charge actually drawn): catches sunny days while the battery
 *    still wants charge;
 *  - Voc sun-hours: hourly open-circuit panel voltage >= SUN_FRACTION_PCT of
 *    the self-calibrating max (NVS-persisted) -- catches sunny days when a
 *    full battery makes harvest blind (the 2026-08-12 finding). Voc is a
 *    daylight detector, not an intensity meter (logarithmic in light,
 *    -0.3 %/C in temperature), hence the loose fraction. */
#define WEATHER_GOOD_MAH        1000
#define SUN_FRACTION_PCT        80
#define SUN_HOURS_GOOD          4
#define TZ_OFFSET_MIN           120     /* local day rollover (CEST) */

/* =========================================================================
 * PRESSURE: WHICH SENSOR MEASURES WHAT
 * =========================================================================
 * GAS pressure  -- the MS5837 on the I2C hat header (sensor_ok bit 3). It sits
 *                  in the GAS LINE, so press_dmbar is gas pressure, NOT ambient.
 *                  Published as p_gas_hpa.
 * ATMOSPHERIC   -- no sensor on this node. Comes from the constant below and is
 *                  published as p_atm_hpa with p_atm_is_const=1 so no analysis
 *                  mistakes it for a reading. SET IT PER SITE: it feeds gas
 *                  density directly.
 *                     sea level (ISA) 1013    Zurich ~400 m 965
 *                     Mzuzu ~1250 m    875    Nairobi ~1795 m 820
 * dp_hpa        -- gas minus atmospheric, published for convenience; both
 *                  inputs are sent so it can be recomputed off-device.
 *
 * The WF280A (U8) on the ultrasonic board was meant to supply gas pressure and
 * does not work: measured 2026-08-15, its status byte reads 0x19 on every
 * access (ADC powered off, test mode set, two reserved-must-be-zero bits set)
 * and a datasheet-correct trigger returns 6. Its compensation polynomial is
 * also unpublished. sensor_ok bit 5 means only that it ACKs, not that its data
 * is usable. A Bosch part replaces it on the respin.
 * ========================================================================= */
#define P_ATM_CONST_HPA         965.0f   /* ATMOSPHERIC, Zurich ~408 m. Per site. */

/* ---- Ultrasonic flow module (MSP430FR6043 I2C slave, uss_link.h) ----
 * A USS measurement is a capture + algorithm run on the MSP430; the reference
 * firmware turns one around well inside 500 ms. Budget 3 s before declaring
 * the module stuck -- a lost measurement costs one sample, a false timeout
 * every 5 min would cost the link. */
/* Autonomous measurement period on the module, seconds. It measures and
 * integrates at this rate continuously; we just read the accumulated volume
 * whenever we wake, so this is decoupled from SAMPLE_INTERVAL_S.
 *
 * 1 Hz is comfortable because the AFE rails stay up for the whole autonomous
 * run -- but that is exactly what makes the module expensive: the OPA836 sits
 * enabled between captures at ~1 mA (~24 mAh/day, the largest single load on
 * the node -- measured 2026-09-02 over 22 days of indoor discharge). The fix
 * belongs on the module (PD-gate the amp per capture, DEVLOG S13), NOT here:
 * once gated a capture costs ~50 uA-s, so this period is worth only ~1-2
 * mAh/day and lengthening it buys ~3% of the node budget. Change it for
 * totalizer integration accuracy, not for power. */
/* Autonomous measurement period, seconds. Back to 1 Hz for the 2026-09-08 soak:
 * measured cost is 84 uC per capture (~84 uA at 1 Hz) against a ~235 uA board,
 * and 1 Hz is what the totalizer wants. 10 s was an experiment to find where the
 * energy went; it turned out to be the gas configuration itself (300 us capture
 * window, multi-tone, OPA836, 5 V boost) -- the water build measures 5.4 uC on
 * the same firmware, near TI's ~3 uC, so there is no hidden fault to chase. */
#define USS_AUTO_PERIOD_S       1

/* USSXT settling time asked of the ultrasonic module, in units of 10 us.
 * 0 = leave the module's own compiled default alone.
 *
 * 800 = 8 ms, which is what the module's XT_SETTLE_RUN_US has been since the
 * 2026-09-05 change -- i.e. this default preserves current behaviour exactly.
 * The module clamps to 1 ms .. 400 ms.
 *
 * This is the knob for the code-135 investigation: zero code-135 in 5656
 * captures before 09-05, then it appears in the hour the settle was cut from
 * 300 ms, and the test that justified the cut was 60 captures against a
 * ~1-in-12000 failure. Raise this to 30000 (300 ms) to test the pre-regression
 * value on a deployed node and watch uss_cap_badcode. */
/* PHASE 1 of the isolation (2026-09-10): 800 = 8 ms, i.e. the settle that is
 * deployed today. Phase 1 changes NOTHING about the settle on purpose -- it
 * measures the one fix that has never actually been rate-tested, the guarded
 * I2C ISR wake, against the known baseline of ~7 episodes/day. Tonight's
 * episodes were all at AUTO_START in the first five minutes and were followed
 * by a single clean record, which is not a rate.
 *
 * Phase 2 sets this to 30000 (300 ms), the settle the firmware ran through its
 * three clean weeks -- ussPLLConfig is __persistent FRAM, so the 2026-09-05 cut
 * to 4 ms has applied to every capture since. Neither phase needs the USS off
 * the comm board; this lives on the master. Confirm which was actually applied
 * via the uss_xt_us telemetry key. */
#define USS_XT_SETTLE_X10US     800

/* Skip deep sleep while a USB cable is attached, so the console stays alive
 * between cycles. USB presence is bq_ac1_present() -- the USB port only; solar
 * comes in on AC2 -- so this can never keep a deployed node awake. Costs
 * nothing on battery. Set to 0 to restore unconditional deep sleep. */
/* Skip deep sleep while a USB cable is attached so the console stays alive.
 * USB presence is bq_ac1_present() -- the USB port only, solar arrives on AC2 --
 * so this can never keep a deployed node awake.
 *
 * NOTE it also suppresses the deep-sleep WAKE entirely, which matters for
 * experiments: an accelerated poll test on 2026-09-10 produced zero hits on
 * both guarded and unguarded USS images because it varied the I2C transaction
 * rate while holding wakes at zero. Set to 0 if the wake is the variable. */
#define USB_STAY_AWAKE          1

/* Poll the ultrasonic module every N seconds during the USB_STAY_AWAKE window,
 * instead of once per SAMPLE_INTERVAL_S. 0 = off (normal behaviour).
 *
 * The code-135 trigger is the poll itself, so this is an accelerated-life test:
 * 5 s instead of 300 s is 60x the event rate, which turns a two-hour rate
 * measurement into about ten minutes. Only meaningful with a cable attached,
 * and only legitimate because the trigger is known rather than guessed. */
/* Off: the poll rate is NOT the variable under test -- see USB_STAY_AWAKE. */
#define USS_POLL_STRESS_S       0

/* Consecutive 5-minute records with EVERY capture failing before the comm board
 * power-cycles the sensor rail. 3 records is ~15 min. See the stuck abs-ToF lock
 * note in main.c. */
#define USS_ALLBAD_RECOVER      3

/* How long to wait for STATUS.AUTO after commanding AUTO_START. The module
 * raises its AFE rails, settles them and runs one discard capture first, so
 * ~700 ms is normal; 3 s is the give-up point. */
#define USS_AUTO_START_TIMEOUT_MS  3000

/* Raised 3000 -> 8000 (2026-09-12). Eleven records in the overnight export
 * consumed the FULL 3 s and still succeeded, i.e. they were grazing the limit;
 * fifteen more tipped over it. A timeout is now cheap (main.c skips the sample
 * instead of cutting the rail), so buy the margin. */
#define USS_MEAS_TIMEOUT_MS     8000
#define USS_POLL_MS             25

/* ---- link characterisation test (2026-09-11) -----------------------------
 * Diagnostic only. Set USS_LINK_TEST_N to 0 for normal operation.
 *
 * Runs at the top of a wake, BEFORE the normal cycle, and classifies every
 * link failure by kind instead of collapsing them into uss_sample()'s single
 * bool. It alternates light sleep off/on between blocks because dynamic
 * frequency scaling and tickless idle were enabled on 2026-09-05 -- the modem
 * was given an ESP_PM_NO_LIGHT_SLEEP lock for exactly this reason and the I2C
 * bus was not -- and because the USS dropping off the bus began somewhere after
 * 2026-09-02, having not happened once in the preceding 18 days.
 *
 * Blocks are INTERLEAVED (A,B,A,B,...) rather than run as two long halves. A
 * monotone sweep on this rig has already produced one false result: a settle
 * "dose-response" on 2026-09-10 that turned out to be warm-up drift, and cost
 * a day before an interleaved A/B showed both arms identical. */
#define USS_LINK_TEST_N         0       /* 0 = off; the console A/B needs a stable cable */
#define USS_LINK_TEST_BLOCKS    3       /* interleaved A/B blocks */

/* Per-transaction retry on the USS link (2026-09-11). One failed transfer used
 * to become "USS silent", whose only remedy is an 800 ms rail power-cycle that
 * reboots the module and zeroes its volume totalizer -- a reboot and a day's
 * volume for one glitch on a 100 kHz bus. */
#define USS_XFER_RETRIES        3
#define USS_XFER_RETRY_MS       5

/* Prior consecutive USS read failures required before the rail is cut.
 * 0 = the old behaviour (recover on the first miss), which produced 17 reboots
 * from 15 isolated failures overnight 2026-09-12 -- every reboot forcing a
 * fresh abs-ToF search and zeroing the volume totalizer. */
#define USS_RECOVER_AFTER_FAILS 1
