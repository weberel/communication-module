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
 * ABSOLUTE PRESSURE: HARD-CODED CONSTANT. THIS IS NOT A MEASUREMENT.
 * =========================================================================
 * The WF280A (U8) on the ultrasonic board does not deliver usable pressure:
 * measured 2026-08-15, its status byte reads 0x19 on every access, which
 * decodes as ADC POWERED OFF (bit6=0) and TEST MODE SET (bit3=1), with two
 * reserved-must-be-zero bits also set. A datasheet-correct trigger (A0 00 00)
 * followed by a 24-bit read returns 6. Its NVM address word reads 0x0000,
 * which per the datasheet means it should answer at 0x78, yet it answers at
 * 0x38 and nothing answers at 0x78.
 *
 * Separately, even a healthy part could not be converted on-device: the
 * WF280A compensation polynomial is NOT published (datasheet 3.10.2 refers you
 * to WF Technologies' driver C code), and it needs 11 per-device NVM
 * coefficients.
 *
 * So until a working pressure sensor is fitted, this constant stands in.
 * It is published under the key "p_abs_const_hpa" and accompanied by
 * "p_abs_is_const":1 so no downstream analysis can mistake it for data.
 * sensor_ok bit5 stays 0.
 *
 * >>> SET THIS PER DEPLOYMENT SITE. Station pressure varies strongly with
 * >>> altitude and it feeds gas-density calculations directly:
 * >>>    sea level (ISA)   1013 hPa
 * >>>    Zurich   ~400 m   ~965 hPa
 * >>>    Mzuzu   ~1250 m   ~875 hPa
 * >>>    Nairobi ~1795 m   ~820 hPa
 * >>> Leaving the sea-level default on a Nairobi node overstates absolute
 * >>> pressure by ~24 %, and gas density with it.
 * ========================================================================= */
#define P_ABS_CONST_HPA         965.0f   /* ZURICH, ~408 m. Change per site. */

/* ---- Ultrasonic flow module (MSP430FR6043 I2C slave, uss_link.h) ----
 * A USS measurement is a capture + algorithm run on the MSP430; the reference
 * firmware turns one around well inside 500 ms. Budget 3 s before declaring
 * the module stuck -- a lost measurement costs one sample, a false timeout
 * every 5 min would cost the link. */
#define USS_MEAS_TIMEOUT_MS     3000
#define USS_POLL_MS             25
