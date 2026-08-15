/*
 * config.h  --  tunables for the ESP-IDF datalogger.
 * Values carried over from the hardware-validated Arduino build
 * (src/datalogger/config.h), plus the field-robustness floors agreed 2026-08-11.
 */
#pragma once

#define FW_VERSION              "idf-0.10"

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
#define MPPT_VOC_PERIOD_WAKES   12
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
#define USS_MEAS_TIMEOUT_MS     3000
#define USS_POLL_MS             25
