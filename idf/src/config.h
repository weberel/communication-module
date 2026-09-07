/*
 * config.h  --  tunables for the ESP-IDF datalogger.
 * Values carried over from the hardware-validated Arduino build
 * (src/datalogger/config.h), plus the field-robustness floors agreed 2026-08-11.
 */
#pragma once

#define FW_VERSION              "idf-0.17"

/* ---- Duty cycle ---- */
#define SAMPLE_INTERVAL_S       300     /* 5 min */
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
/* Autonomous measurement period, seconds.
 *
 * 1 -> 10 as an EXPERIMENT (2026-09-06): the power audit's differential is
 * AUTO-running vs AUTO-stopped, so the 1665 +/- 139 uA it measures includes the
 * whole energy cost of the captures themselves, not just a static rail load.
 * The RX bias divider is 2M/2M = 0.8 uA and the other static fixes total
 * ~100 uA, so static cannot explain 1.665 mA. If the cost is the captures,
 * measuring 10x less often should take the figure toward ~170 uA. If it stays
 * put, the load is static after all and only the rail FETs can reach it. */
#define USS_AUTO_PERIOD_S       10

/* How long to wait for STATUS.AUTO after commanding AUTO_START. The module
 * raises its AFE rails, settles them and runs one discard capture first, so
 * ~700 ms is normal; 3 s is the give-up point. */
#define USS_AUTO_START_TIMEOUT_MS  3000

#define USS_MEAS_TIMEOUT_MS     3000
#define USS_POLL_MS             25
