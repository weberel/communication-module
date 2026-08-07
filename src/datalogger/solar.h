/*
 * solar.h  --  solar charging manager: MPPT + weather-adaptive charge target.
 *
 * MPPT (maximise input current/power):
 *   The BQ25792 has no hardware MPPT; its VINDPM loop is our knob. Every wake with
 *   solar as the input we
 *     1. periodically (or when the panel has collapsed) measure the panel's
 *        open-circuit voltage by putting the charger in HIZ for a moment, and jump
 *        VINDPM to FOC_PCT (~80 %) of Voc -- the fractional-open-circuit method
 *        lands within a few % of the true MPP in one step, whatever the weather;
 *     2. run a few perturb-&-observe steps around that point, keeping whichever
 *        direction increases measured input power (VBUS x IBUS).
 *   The operating point lives in RTC RAM, so tracking continues seamlessly across
 *   deep sleep. On USB power VINDPM is simply parked at USB_VINDPM_MV.
 *
 * Weather / 80 % state of charge:
 *   Solar charge harvested is integrated per local calendar day. If yesterday's or
 *   today's harvest reaches WEATHER_GOOD_MAH the weather counts as "good" and the
 *   charge target drops to VREG_ECO_MV (~80 % SoC) to spare the LiPo from sitting
 *   at 4.2 V; in bad weather it returns to VREG_FULL_MV to build maximum reserve.
 *   Enforced in hardware via the charger's VREG register, so it holds while the
 *   ESP32 sleeps.
 */
#pragma once

#include <Arduino.h>
#include "BQ25792.h"

namespace Solar {

struct Status {
    bool     solar_present;
    bool     usb_present;
    bool     weather_good;
    bool     eco_target;        /* charging capped at ~80 % SoC */
    uint16_t vindpm_mv;
    uint16_t vreg_mv;
    uint16_t harvest_today_mah;
    uint16_t harvest_prev_mah;
};

/* Reset all tracking state (cold boot). */
void reset();

/* Run one management pass: input limits, MPPT step, harvest accounting, day
 * rollover, charge-target selection. Call once per wake, after bq is up with its
 * ADC running. day_num = localDayNum() (or -1 while the clock is unsynced). */
Status onWake(BQ25792& bq, uint32_t interval_s, int32_t day_num);

}  // namespace Solar
