/*
 * ecoTrace datalogger
 * ===================
 * A working starting point. Fork it and change whatever you need - this is your app,
 * not a framework. Each wake it:
 *   1. reads the battery + charger state (BQ25792) and a sensor (the on-board
 *      LTR-303 light sensor, as an example),
 *   2. keeps the battery charging from whatever input is present (USB or solar, with
 *      a software MPPT step for solar),
 *   3. buffers the sample in RTC RAM (survives deep sleep),
 *   4. every SAMPLES_PER_UPLOAD samples, powers the A7672E modem and uploads the
 *      buffer over cellular (HTTP POST, one JSON object per record),
 *   5. deep-sleeps until the next sample.
 *
 * To log YOUR sensor: change readSample() (read it), the LogRecord struct (store it),
 * and buildJson() (send it). Drivers for the on-board parts are in lib/EcoTrace; see
 * docs/api-reference.md. Connect external sensors on the SENSOR (I2C) or SPI header.
 *
 * Config lives in config.h. Credentials (SIM_APN, POST_URL) live in secrets.h.
 *
 * Build/flash:  pio run -e datalogger -t upload && pio device monitor
 */
#include <Arduino.h>
#include "EcoTraceBoard.h"
#include "BQ25792.h"
#include "LTR303.h"
#include "ModemA7672.h"
#include "config.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef SIM_APN
#define SIM_APN "internet"
#endif

/* ===================== record + RTC-persistent state ===================== */
/* One sample. Add/remove fields here to log what you want. Kept in RTC RAM, so it
 * survives deep sleep but is lost on a full power cut. */
typedef struct {
    uint32_t uptime_s;     /* board uptime at sample time (for timestamping) */
    uint16_t vbat_mv;
    int16_t  ibat_ma;      /* + charging, - discharging */
    uint16_t vbus_mv;
    int16_t  ibus_ma;
    uint8_t  chg_stat;     /* BQ25792::ChgStat */
    uint8_t  fault0;
    uint8_t  fault1;
    int8_t   soc_pct;      /* crude voltage-based estimate */
    uint16_t light_ch0;    /* EXAMPLE sensor: LTR-303 visible+IR. Replace with yours. */
} LogRecord;

static RTC_DATA_ATTR LogRecord s_buf[BUFFER_SIZE];
static RTC_DATA_ATTR uint16_t  s_count;
static RTC_DATA_ATTR uint16_t  s_samples_since_upload;
static RTC_DATA_ATTR uint32_t  s_uptime_s;
static RTC_DATA_ATTR uint16_t  s_vindpm_mv;
static RTC_DATA_ATTR int8_t    s_mppt_dir;
static RTC_DATA_ATTR int32_t   s_mppt_last_pwr;

BQ25792    bq;
LTR303     light;
ModemA7672 modem;

/* ===================== helpers ===================== */

/* Very crude 1S LiPo SoC from resting voltage. Replace with an OCV table or a
 * coulomb counter if you need accuracy; under load this reads low. */
static int8_t socFromVoltage(uint16_t vbat_mv)
{
    if (vbat_mv >= 4200) return 100;
    if (vbat_mv <= 3300) return 0;
    return (int8_t)((vbat_mv - 3300) * 100 / (4200 - 3300));
}

/* One MPPT hill-climb step. Call only when charging from solar (VAC2). */
static void mpptStep()
{
    int32_t vbus = bq.readVbus_mV();
    int32_t ibus = bq.readIbus_mA();
    int32_t pwr  = vbus * (ibus > 0 ? ibus : 0);
    if (s_mppt_last_pwr >= 0 && pwr < s_mppt_last_pwr) s_mppt_dir = -s_mppt_dir;
    int32_t next = (int32_t)s_vindpm_mv + s_mppt_dir * MPPT_VINDPM_STEP_MV;
    if (next < MPPT_VINDPM_MIN_MV) { next = MPPT_VINDPM_MIN_MV; s_mppt_dir = +1; }
    if (next > MPPT_VINDPM_MAX_MV) { next = MPPT_VINDPM_MAX_MV; s_mppt_dir = -1; }
    s_vindpm_mv     = (uint16_t)next;
    s_mppt_last_pwr = pwr;
    bq.setVINDPM_mV(s_vindpm_mv);
}

/* Fill one record from the sensors. */
static void readSample(LogRecord& r)
{
    r.uptime_s = s_uptime_s;

    /* Battery + charger (always logged). */
    r.vbat_mv  = bq.readVbat_mV();
    r.ibat_ma  = bq.readIbat_mA();
    r.vbus_mv  = bq.readVbus_mV();
    r.ibus_ma  = bq.readIbus_mA();
    r.chg_stat = (uint8_t)bq.chargeState();
    bq.readFaults(r.fault0, r.fault1);
    r.soc_pct  = socFromVoltage(r.vbat_mv);

    /* --- YOUR SENSOR --------------------------------------------------------
     * Example: the on-board LTR-303 light sensor. Swap this for whatever you
     * attached (I2C on the SENSOR header, SPI, an analog pin, ...). Use
     * EcoTrace::sensorRail(true/false) if it is on the switched sensor rail. */
    r.light_ch0 = light.begin() ? light.readCh0() : 0;
    light.powerDown();
}

/* Build the telemetry JSON for one record. ts (unix ms) is added when we have
 * network time; otherwise the server timestamps on receipt. */
static void buildJson(const LogRecord& r, int64_t now_ms, uint32_t uptime_now,
                      char* out, size_t len)
{
    float pwr_mw = (float)r.vbat_mv / 1000.0f * (float)r.ibat_ma;
    char values[320];
    snprintf(values, sizeof(values),
        "\"vbat_mv\":%u,\"ibat_ma\":%d,\"bat_power_mw\":%.0f,\"soc_pct\":%d,"
        "\"vbus_mv\":%u,\"ibus_ma\":%d,\"chg_stat\":%u,\"fault0\":%u,\"fault1\":%u,"
        "\"light_ch0\":%u",                       /* <- your sensor field(s) here */
        r.vbat_mv, r.ibat_ma, pwr_mw, r.soc_pct,
        r.vbus_mv, r.ibus_ma, r.chg_stat, r.fault0, r.fault1,
        r.light_ch0);

    if (now_ms > 0) {
        int64_t ts = now_ms - (int64_t)(uptime_now - r.uptime_s) * 1000;
        snprintf(out, len, "{\"ts\":%lld,\"values\":{%s}}", (long long)ts, values);
    } else {
        snprintf(out, len, "{%s}", values);
    }
}

/* Power the modem, attach, POST every buffered record, clear on success. */
static void uploadBuffer()
{
#ifdef POST_URL
    Serial.println("Upload: powering modem...");
    if (!modem.begin()) { Serial.println("  modem did not boot"); modem.powerOff(); return; }
    if (!modem.simReady() || !modem.waitForNetwork(30000)) {
        Serial.println("  no SIM / not registered"); modem.powerOff(); return;
    }
    if (!modem.connectGPRS(SIM_APN)) { Serial.println("  PDP failed"); modem.powerOff(); return; }
    Serial.printf("  attached, %d dBm\n", modem.signalQuality_dBm());

    int64_t now_ms = modem.getUnixTimeMs();   /* 0 if network time unavailable */

    uint16_t sent = 0;
    for (uint16_t i = 0; i < s_count; i++) {
        char json[420];
        buildJson(s_buf[i], now_ms, s_uptime_s, json, sizeof(json));
        int status;
        if (!modem.httpPost(POST_URL, "application/json", json, status)) {
            Serial.printf("  POST failed at record %u (HTTP %d)\n", i, status);
            break;
        }
        sent++;
    }
    Serial.printf("  uploaded %u/%u records\n", sent, s_count);
    if (sent == s_count) s_count = 0;         /* clear only on full success */
    modem.powerOff();
#else
    Serial.println("Upload skipped: POST_URL not set in secrets.h (bench mode).");
    s_count = 0;   /* drop the buffer so it doesn't overflow on the bench */
#endif
}

/* ===================== main ===================== */
void setup()
{
    Serial.begin(115200);
    delay(1500);
    EcoTrace::beginBoard();
    EcoTrace::beginI2C();

    bool first_boot = !EcoTrace::wokeFromTimer();
    if (first_boot) {
        s_count = 0;
        s_samples_since_upload = 0;
        s_uptime_s = 0;
        s_vindpm_mv = MPPT_VINDPM_START_MV;
        s_mppt_dir = +1;
        s_mppt_last_pwr = -1;
    } else {
        s_uptime_s += SAMPLE_INTERVAL_S;   /* approx: the sleep we just finished */
    }

    /* Charger: bring up ADC, keep charging configured (watchdog disabled so our
     * settings persist). Cheap to redo every wake. */
    if (bq.begin()) {
        bq.enableADC();
        bq.enableIbatSensing();
        bq.configureCharging(CHARGE_CURRENT_MA, INPUT_LIMIT_MA, CHARGE_VOLTAGE_MV);
        bq.enableACDRV1(true);   /* USB input sits behind ACFET1, which is off at POR */
#if MPPT_ENABLE
        bq.enableACDRV2(true);
        bq.setVINDPM_mV(s_vindpm_mv);
        if (bq.ac2Present()) mpptStep();   /* only track when solar is the source */
#endif
    } else {
        Serial.println("WARNING: BQ25792 not found -- battery data will be zero.");
    }

    /* Sample. */
    if (s_count < BUFFER_SIZE) {
        readSample(s_buf[s_count]);
        LogRecord& r = s_buf[s_count];
        Serial.printf("sample %u: VBAT=%u mV IBAT=%+d mA SOC=%d%% VBUS=%u mV %s light=%u\n",
                      s_count, r.vbat_mv, r.ibat_ma, r.soc_pct, r.vbus_mv,
                      bq.chargeStateName(), r.light_ch0);
        s_count++;
    } else {
        Serial.println("buffer full -- forcing upload");
        s_samples_since_upload = SAMPLES_PER_UPLOAD;
    }

    /* Upload if due. */
    if (++s_samples_since_upload >= SAMPLES_PER_UPLOAD) {
        s_samples_since_upload = 0;
        uploadBuffer();
    }

    if (bq.isPresent()) bq.disableADC();   /* save the charger's ADC power in sleep */

    Serial.printf("sleeping %d s (buffered %u/%u)\n",
                  SAMPLE_INTERVAL_S, s_count, BUFFER_SIZE);
    Serial.flush();
    EcoTrace::deepSleepSeconds(SAMPLE_INTERVAL_S);   /* never returns */
}

void loop() {}
