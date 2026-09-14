/*
 * solar.h  --  solar charging manager, ESP-IDF port.
 * Logic identical to the Arduino module (src/datalogger/solar.*): fractional-Voc
 * + P&O MPPT on VINDPM, per-day harvest accounting, weather-adaptive charge
 * target (good weather -> ~80 % SoC cap; USB always charges full).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     solar_present;
    bool     usb_present;
    bool     weather_good;
    bool     eco_target;
    uint16_t vindpm_mv;
    uint16_t vreg_mv;
    uint16_t harvest_today_mah;
    uint16_t harvest_prev_mah;
    uint8_t  sun_hours;          /* Voc-above-threshold hours so far today */
    uint16_t voc_max_mv;         /* self-calibrating panel Voc reference (NVS) */
} solar_status_t;

void solar_reset(void);   /* cold boot */

/* One management pass per wake; bq must be up with its ADC running.
 * day_num = local day number, or -1 while the clock is unsynced. */
solar_status_t solar_on_wake(uint32_t interval_s, int32_t day_num);

#ifdef __cplusplus
}
#endif
