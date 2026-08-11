#include "solar.h"
#include "config.h"
#include "bq25792.h"

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static RTC_DATA_ATTR uint16_t s_vindpm_mv;
static RTC_DATA_ATTR int8_t   s_dir;
static RTC_DATA_ATTR uint16_t s_wakes_since_voc;
static RTC_DATA_ATTR uint32_t s_day_mas;        /* today's harvest, mA-seconds */
static RTC_DATA_ATTR uint32_t s_prev_day_mas;
static RTC_DATA_ATTR int32_t  s_day_num;
static RTC_DATA_ATTR uint32_t s_day_elapsed_s;

void solar_reset(void)
{
    s_vindpm_mv       = MPPT_VINDPM_START_MV;
    s_dir             = +1;
    s_wakes_since_voc = MPPT_VOC_PERIOD_WAKES;   /* force Voc on first solar wake */
    s_day_mas         = 0;
    s_prev_day_mas    = 0;
    s_day_num         = -1;
    s_day_elapsed_s   = 0;
}

static uint16_t clamp_vindpm(int32_t mv)
{
    if (mv < MPPT_VINDPM_MIN_MV) return MPPT_VINDPM_MIN_MV;
    if (mv > MPPT_VINDPM_MAX_MV) return MPPT_VINDPM_MAX_MV;
    return (uint16_t)mv;
}

static int32_t input_power_mw(void)
{
    int32_t acc = 0;
    for (int i = 0; i < 3; i++) {
        int32_t ibus = bq_ibus_ma();
        if (ibus < 0) ibus = 0;
        acc += (int32_t)bq_vbus_mv() * ibus / 1000;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return acc / 3;
}

static void mppt_step(void)
{
    bool need_voc = (++s_wakes_since_voc >= MPPT_VOC_PERIOD_WAKES);
    if (bq_ibus_ma() < MPPT_COLLAPSE_MA) need_voc = true;

    if (need_voc && bq_vbat_mv() > 3400) {
        bq_set_hiz(true);
        vTaskDelay(pdMS_TO_TICKS(MPPT_VOC_SETTLE_MS));
        uint16_t voc = bq_vac2_mv();
        bq_set_hiz(false);
        s_wakes_since_voc = 0;
        if (voc > MPPT_VINDPM_MIN_MV) {
            s_vindpm_mv = clamp_vindpm((int32_t)voc * MPPT_FOC_PCT / 100);
            bq_set_vindpm_mv(s_vindpm_mv);
            vTaskDelay(pdMS_TO_TICKS(MPPT_SETTLE_MS));
        }
    }

    bq_set_vindpm_mv(s_vindpm_mv);
    int32_t last_mw = -1;
    for (int i = 0; i < MPPT_STEPS_PER_WAKE; i++) {
        vTaskDelay(pdMS_TO_TICKS(MPPT_SETTLE_MS));
        int32_t mw = input_power_mw();
        if (last_mw >= 0 && mw < last_mw) s_dir = -s_dir;
        int32_t next = (int32_t)s_vindpm_mv + s_dir * MPPT_STEP_MV;
        if (next <= MPPT_VINDPM_MIN_MV) { next = MPPT_VINDPM_MIN_MV; s_dir = +1; }
        if (next >= MPPT_VINDPM_MAX_MV) { next = MPPT_VINDPM_MAX_MV; s_dir = -1; }
        s_vindpm_mv = (uint16_t)next;
        bq_set_vindpm_mv(s_vindpm_mv);
        last_mw = mw;
    }
}

solar_status_t solar_on_wake(uint32_t interval_s, int32_t day_num)
{
    solar_status_t st = { 0 };
    st.usb_present   = bq_ac1_present();
    st.solar_present = bq_ac2_present();

    if (st.usb_present) {
        bq_set_iindpm_ma(INPUT_LIMIT_USB_MA);
        bq_set_vindpm_mv(USB_VINDPM_MV);
    } else if (st.solar_present) {
        bq_set_iindpm_ma(INPUT_LIMIT_SOLAR_MA);
        mppt_step();
    }

    if (st.solar_present && !st.usb_present) {
        int32_t ibus = bq_ibus_ma();
        if (ibus > 0) s_day_mas += (uint32_t)ibus * interval_s;
    }

    bool rolled = false;
    if (day_num >= 0) {
        if (s_day_num >= 0 && day_num != s_day_num) rolled = true;
        s_day_num = day_num;
    } else {
        s_day_elapsed_s += interval_s;
        if (s_day_elapsed_s >= 86400) { s_day_elapsed_s = 0; rolled = true; }
    }
    if (rolled) { s_prev_day_mas = s_day_mas; s_day_mas = 0; }

    uint16_t today_mah = (uint16_t)(s_day_mas / 3600);
    uint16_t prev_mah  = (uint16_t)(s_prev_day_mas / 3600);
    st.weather_good = (prev_mah >= WEATHER_GOOD_MAH) || (today_mah >= WEATHER_GOOD_MAH);
    /* USB always charges full: plugging a cable in is a deliberate "fill it up". */
    st.eco_target = st.weather_good && !st.usb_present;

    uint16_t vreg = st.eco_target ? VREG_ECO_MV : VREG_FULL_MV;
    bq_set_vreg_mv(vreg);

    st.vindpm_mv         = s_vindpm_mv;
    st.vreg_mv           = vreg;
    st.harvest_today_mah = today_mah;
    st.harvest_prev_mah  = prev_mah;
    return st;
}
