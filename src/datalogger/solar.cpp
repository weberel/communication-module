#include "solar.h"
#include "config.h"

/* All tracking state survives deep sleep in RTC RAM. */
static RTC_DATA_ATTR uint16_t s_vindpm_mv;
static RTC_DATA_ATTR int8_t   s_dir;
static RTC_DATA_ATTR uint16_t s_wakes_since_voc;
static RTC_DATA_ATTR uint32_t s_day_mas;        /* today's harvest, milliamp-seconds */
static RTC_DATA_ATTR uint32_t s_prev_day_mas;   /* yesterday's harvest */
static RTC_DATA_ATTR int32_t  s_day_num;        /* local day of s_day_mas, -1 = counter mode */
static RTC_DATA_ATTR uint32_t s_day_elapsed_s;  /* day tracking while clock unsynced */

namespace Solar {

void reset()
{
    s_vindpm_mv       = MPPT_VINDPM_START_MV;
    s_dir             = +1;
    s_wakes_since_voc = MPPT_VOC_PERIOD_WAKES;   /* force a Voc measurement first wake */
    s_day_mas         = 0;
    s_prev_day_mas    = 0;
    s_day_num         = -1;
    s_day_elapsed_s   = 0;
}

static uint16_t clampVindpm(int32_t mv)
{
    if (mv < MPPT_VINDPM_MIN_MV) return MPPT_VINDPM_MIN_MV;
    if (mv > MPPT_VINDPM_MAX_MV) return MPPT_VINDPM_MAX_MV;
    return (uint16_t)mv;
}

/* Averaged input power in mW (VBUS x IBUS); the 15-bit ADC runs continuously. */
static int32_t inputPower_mW(BQ25792& bq)
{
    int32_t acc = 0;
    for (int i = 0; i < 3; i++) {
        int32_t ibus = bq.readIbus_mA();
        if (ibus < 0) ibus = 0;
        acc += (int32_t)bq.readVbus_mV() * ibus / 1000;
        delay(20);
    }
    return acc / 3;
}

static void mpptStep(BQ25792& bq)
{
    /* Fractional-Voc re-anchor: periodically, or whenever the panel has collapsed
     * (dawn, passing cloud, VINDPM stuck way off the new MPP). */
    bool need_voc = (++s_wakes_since_voc >= MPPT_VOC_PERIOD_WAKES);
    if (bq.readIbus_mA() < MPPT_COLLAPSE_MA) need_voc = true;

    if (need_voc && bq.readVbat_mV() > 3400) {   /* HIZ = run on battery for a moment */
        bq.setHIZ(true);
        delay(MPPT_VOC_SETTLE_MS);
        uint16_t voc = bq.readVac2_mV();
        bq.setHIZ(false);
        s_wakes_since_voc = 0;
        if (voc > MPPT_VINDPM_MIN_MV) {
            s_vindpm_mv = clampVindpm((int32_t)voc * MPPT_FOC_PCT / 100);
            bq.setVINDPM_mV(s_vindpm_mv);
            delay(MPPT_SETTLE_MS);   /* let the input loop re-engage */
        }
    }

    /* Perturb & observe around the anchor: keep stepping in whichever direction
     * input power went up. State carries over to the next wake. */
    bq.setVINDPM_mV(s_vindpm_mv);
    int32_t last_mw = -1;
    for (int i = 0; i < MPPT_STEPS_PER_WAKE; i++) {
        delay(MPPT_SETTLE_MS);
        int32_t mw = inputPower_mW(bq);
        if (last_mw >= 0 && mw < last_mw) s_dir = -s_dir;
        int32_t next = (int32_t)s_vindpm_mv + s_dir * MPPT_STEP_MV;
        if (next <= MPPT_VINDPM_MIN_MV) { next = MPPT_VINDPM_MIN_MV; s_dir = +1; }
        if (next >= MPPT_VINDPM_MAX_MV) { next = MPPT_VINDPM_MAX_MV; s_dir = -1; }
        s_vindpm_mv = (uint16_t)next;
        bq.setVINDPM_mV(s_vindpm_mv);
        last_mw = mw;
    }
}

Status onWake(BQ25792& bq, uint32_t interval_s, int32_t day_num)
{
    Status st = {};
    st.usb_present   = bq.ac1Present();
    st.solar_present = bq.ac2Present();

    /* Input limits: current limit high enough to never be the bottleneck (the
     * panel limits itself; USB per config), VINDPM per source. */
    if (st.usb_present) {
        bq.setInputCurrentLimit_mA(INPUT_LIMIT_USB_MA);
        bq.setVINDPM_mV(USB_VINDPM_MV);
    } else if (st.solar_present) {
        bq.setInputCurrentLimit_mA(INPUT_LIMIT_SOLAR_MA);
        mpptStep(bq);
    }

    /* Harvest accounting: charge pulled from the input while on solar, integrated
     * over the sample interval. */
    if (st.solar_present && !st.usb_present) {
        int32_t ibus = bq.readIbus_mA();
        if (ibus > 0) s_day_mas += (uint32_t)ibus * interval_s;
    }

    /* Day rollover: by local calendar day once the clock is synced, else by a
     * plain 24 h elapsed counter. */
    bool rolled = false;
    if (day_num >= 0) {
        if (s_day_num >= 0 && day_num != s_day_num) rolled = true;
        s_day_num = day_num;
    } else {
        s_day_elapsed_s += interval_s;
        if (s_day_elapsed_s >= 86400) { s_day_elapsed_s = 0; rolled = true; }
    }
    if (rolled) { s_prev_day_mas = s_day_mas; s_day_mas = 0; }

    /* Weather call + charge target. "Good" if yesterday delivered, or today
     * already has -- so the cap engages the moment a sunny day proves itself.
     * USB always charges to 100 %: plugging in a cable is a deliberate act
     * ("fill it up"), and the longevity cap only makes sense on solar, where
     * the energy keeps coming tomorrow. */
    uint16_t today_mah = (uint16_t)(s_day_mas / 3600);
    uint16_t prev_mah  = (uint16_t)(s_prev_day_mas / 3600);
    st.weather_good = (prev_mah >= WEATHER_GOOD_MAH) || (today_mah >= WEATHER_GOOD_MAH);
    st.eco_target   = st.weather_good && !st.usb_present;

    uint16_t vreg = st.eco_target ? VREG_ECO_MV : VREG_FULL_MV;
    bq.setChargeVoltage_mV(vreg);

    st.vindpm_mv         = s_vindpm_mv;
    st.vreg_mv           = vreg;
    st.harvest_today_mah = today_mah;
    st.harvest_prev_mah  = prev_mah;
    return st;
}

}  // namespace Solar
