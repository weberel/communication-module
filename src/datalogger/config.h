/*
 * datalogger/config.h  --  tunables for the ecoTrace datalogger.
 * Edit these to change the duty cycle, charge profile, and upload target.
 */
#pragma once

/* ---- Duty cycle --------------------------------------------------------------
 * The board wakes every SAMPLE_INTERVAL_S, records one sample, and deep-sleeps.
 * Every SAMPLES_PER_UPLOAD samples it powers the modem and uploads the buffer. */
#define SAMPLE_INTERVAL_S     60      /* seconds of deep sleep between samples */
#define SAMPLES_PER_UPLOAD    15      /* upload every N samples (15 x 60 s = 15 min) */
#define BUFFER_SIZE           48      /* max records held in RTC RAM between uploads */

/* ---- Charge profile (BQ25792) ----------------------------------------------- */
#define CHARGE_VOLTAGE_MV     4200    /* 1S LiPo full-charge target (VREG) */
#define CHARGE_CURRENT_MA     1000    /* ICHG */
#define INPUT_LIMIT_MA        2000    /* IINDPM */

/* ---- Solar MPPT (software, via VINDPM hill-climb) ----------------------------
 * The BQ25792 has no true MPPT; we track the panel's max-power point by nudging
 * the input-voltage-regulation threshold (VINDPM) once per wake. Set the bounds
 * around your panel's Vmp / Voc. */
#define MPPT_ENABLE           1
#define MPPT_VINDPM_START_MV  18000
#define MPPT_VINDPM_MIN_MV    14000
#define MPPT_VINDPM_MAX_MV    21000
#define MPPT_VINDPM_STEP_MV   200

/* ---- Upload -----------------------------------------------------------------
 * SIM_APN and POST_URL come from secrets.h (copy secrets.example.h). If POST_URL
 * is not defined the datalogger runs and prints records over serial but does not
 * transmit (useful for bench testing without a SIM). */
