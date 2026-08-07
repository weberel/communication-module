/*
 * Communication Module - Functionality Test (Arduino port)
 * ---------------------------------------------------------
 * Target  : ESP32-C6-MINI-1 (Arduino-ESP32 v3+)
 * Output  : USB Serial/JTAG (GPIO12/13) @ any baud
 *
 * Direct port of the ESP-IDF version. Same test sequence, same
 * [PASS]/[FAIL]/[SKIP]/[INFO] output format, same buffered replay
 * to guard against early-USB-CDC drops on the host.
 */

#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <stdarg.h>

/* ===================== Pin map ===================== */
#define PIN_I2C_SDA          6
#define PIN_I2C_SCL          7

#define PIN_SPI_MOSI         4
#define PIN_SPI_MISO         5
/* Overridable for per-unit rework: unit 8C:FD:49:05:CA:AC has open ESP-module
 * lands on IO8 (flash CS) and IO18 (SCLK) and runs with a 2-wire bypass -
 * build it with: -DPIN_SPI_CS_FLASH=1 -DPIN_SPI_CLK=9
 * (CS via a TP410 wire to U602 pin 1; clock via a header jumper CS<->SCLK,
 * which suspends the CS2 test since GPIO9 is borrowed as clock.) */
#ifndef PIN_SPI_CLK
#define PIN_SPI_CLK          18
#endif
#ifndef PIN_SPI_CS_FLASH
#define PIN_SPI_CS_FLASH     8
#endif
#define PIN_SPI_CS_PERIPH    9

#define PIN_MODEM_TX         20    /* ESP TX -> modem RX */
#define PIN_MODEM_RX         21    /* ESP RX <- modem TX */
#define PIN_MODEM_PWRKEY     22
#define PIN_MODEM_PWR_EN     23
#define PIN_MODEM_STATUS     0     /* strapping pin - read briefly only */

#define PIN_QON              2
#define PIN_INT_SHARED       3     /* SC7A20 + LTR-303 shared INT */
#define PIN_INT_BQ           19

#define PIN_SENSOR_PWR       14    /* external sensor rail high-side switch (populated), active HIGH */
#define PIN_LED              1     /* status LED, blinks during test */

/* ===================== I2C device addresses (7-bit) ===================== */
#define ADDR_SC7A20_LO       0x18
#define ADDR_SC7A20_HI       0x19
#define ADDR_LTR303          0x29
#define ADDR_MS5837          0x76    /* MS5837-02BA barometer (SENSOR header) */
#define ADDR_ATECC608B_A     0x60
#define ADDR_ATECC608B_B     0x35
#define ADDR_BQ25792         0x6B

/* ===================== Bus / test config ===================== */
#define I2C_FREQ_HZ          100000
#define SPI_FREQ_HZ          1000000
#define MODEM_BAUD           115200
#define MODEM_PWRKEY_ON_MS   600
#define MODEM_PWRKEY_OFF_MS  3500

/* Credentials come from secrets.h (copy secrets.example.h -> secrets.h).
 * Fallback placeholders keep this compiling if you have not made one yet;
 * the WiFi + APN sub-tests will simply fail until you set real values. */
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID            "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS            "YOUR_PASSWORD"
#endif
#ifndef SIM_APN
#define SIM_APN              "internet"
#endif
#define WIFI_CONNECT_MS      20000
#define MODEM_APN            SIM_APN
#define HTTP_TEST_URL        "http://www.google.com/generate_204"

#define LED_BLINK_PERIOD_MS  200
#define LED_TAIL_BLINK_MS    5000

/* ===================== State ===================== */
static int    g_pass = 0, g_fail = 0, g_skip = 0;
static char   s_log_buf[6144];
static size_t s_log_len = 0;
static uint8_t s_sc7a20_addr = 0;
static uint8_t s_atecc_addr  = 0;
static volatile bool s_led_blink = false;

static HardwareSerial &modem = Serial1;

/* ===================== Logging ===================== */
static void log_emit(const char *line, int n)
{
    Serial.write((const uint8_t *)line, n);
    Serial.flush();
    if (s_log_len + n < (int)sizeof(s_log_buf)) {
        memcpy(s_log_buf + s_log_len, line, n);
        s_log_len += n;
    }
}

static void test_log(const char *kind, const char *name, const char *fmt, ...)
{
    char line[256];
    int n = snprintf(line, sizeof(line), "[%s] %-14s : ", kind, name);
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(line + n, sizeof(line) - n - 2, fmt, ap);
    va_end(ap);
    if (n >= (int)sizeof(line) - 1) n = sizeof(line) - 2;
    line[n++] = '\n';
    line[n]   = 0;
    log_emit(line, n);
}

static void section_log(const char *title)
{
    char line[160];
    int n = snprintf(line, sizeof(line), "\n--- %s ---\n", title);
    log_emit(line, n);
}

#define TEST_PASS(name, fmt, ...) do { g_pass++; test_log("PASS", (name), (fmt), ##__VA_ARGS__); } while (0)
#define TEST_FAIL(name, fmt, ...) do { g_fail++; test_log("FAIL", (name), (fmt), ##__VA_ARGS__); } while (0)
#define TEST_SKIP(name, fmt, ...) do { g_skip++; test_log("SKIP", (name), (fmt), ##__VA_ARGS__); } while (0)
#define TEST_INFO(name, fmt, ...) do {            test_log("INFO", (name), (fmt), ##__VA_ARGS__); } while (0)
#define SECTION(title)            do {            section_log((title));                          } while (0)

/* ===================== LED status (FreeRTOS task) ===================== */
static void led_blink_task(void *arg)
{
    int level = 0;
    while (s_led_blink) {
        level ^= 1;
        digitalWrite(PIN_LED, level);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS / 2));
    }
    vTaskDelete(NULL);
}

static void led_start(void)
{
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    s_led_blink = true;
    xTaskCreate(led_blink_task, "led", 2048, NULL, 1, NULL);
}

static void led_stop_solid_on(void)
{
    s_led_blink = false;
    delay(LED_BLINK_PERIOD_MS);
    digitalWrite(PIN_LED, HIGH);
}

/* ===================== Power rails ===================== */
static void test_power_rails(void)
{
    SECTION("Power rails");
    /* GPIO14 sensor-rail MOSFET is populated (active HIGH). The on-board SC7A20 +
     * LTR-303 are on 3V3, not this rail, so they stay powered regardless. */
    pinMode(PIN_MODEM_PWR_EN, OUTPUT);
    digitalWrite(PIN_MODEM_PWR_EN, LOW);
    pinMode(PIN_MODEM_PWRKEY, OUTPUT);
    digitalWrite(PIN_MODEM_PWRKEY, HIGH);
    TEST_INFO("SENSOR_PWR",   "GPIO%d ext-sensor rail switch (populated, off); feeds J404 only -- the I2C header J401 is on always-on 3V3", PIN_SENSOR_PWR);
    TEST_INFO("MODEM_PWR_EN", "GPIO%d -> LOW  (modem rail off)", PIN_MODEM_PWR_EN);
    TEST_INFO("MODEM_PWRKEY", "GPIO%d -> HIGH (idle)",           PIN_MODEM_PWRKEY);
}

/* ===================== I2C helpers ===================== */
static bool i2c_probe(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission(true) == 0;
}

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

/* ATECC608B wake: a 0x00-address transaction holds SDA low long enough to wake the chip */
static void atecc_wake(void)
{
    Wire.beginTransmission(0x00);
    Wire.endTransmission(true);
    delay(2);   /* tWHI */
}

static void test_i2c_scan(void)
{
    SECTION("I2C bus scan (GPIO6=SDA, GPIO7=SCL)");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (!i2c_probe(a)) continue;
        const char *name = "(unknown)";
        if      (a == ADDR_SC7A20_LO || a == ADDR_SC7A20_HI) { name = "SC7A20";     s_sc7a20_addr = a; }
        else if (a == ADDR_LTR303)                             name = "LTR-303ALS";
        else if (a == ADDR_ATECC608B_A)                        name = "ATECC608B?";
        else if (a == ADDR_ATECC608B_B)                        name = "ATECC608B?";
        else if (a == ADDR_BQ25792)                            name = "BQ25792";
        else if (a == ADDR_MS5837)                             name = "MS5837-02BA";
        char line[48];
        int n = snprintf(line, sizeof(line), "  0x%02X  %s\n", a, name);
        log_emit(line, n);
        found++;
    }
    if (found == 0) {
        TEST_FAIL("I2C scan", "no devices ACKed - pullups, sensor power, soldering");
    } else {
        TEST_INFO("I2C scan", "%d device(s) responded", found);
    }
}

/* ===================== ATECC608B ===================== */
static void test_atecc608b(void)
{
    atecc_wake();
    bool a60 = i2c_probe(ADDR_ATECC608B_A);
    if (!a60) atecc_wake();
    bool a35 = i2c_probe(ADDR_ATECC608B_B);
    if      (a60) s_atecc_addr = ADDR_ATECC608B_A;
    else if (a35) s_atecc_addr = ADDR_ATECC608B_B;

    if (s_atecc_addr) TEST_PASS("ATECC608B", "ACK at 0x%02X (after wake pulse)", s_atecc_addr);
    else              TEST_FAIL("ATECC608B", "no ACK at 0x60 or 0x35 even after wake");
}

/* ===================== BQ25792 ===================== */
static const char *chg_names[] = {
    "not charging", "trickle", "pre-charge", "fast (CC)",
    "taper (CV)",  "reserved", "top-off",    "done"
};

/* ILIM_HIZ probe (repo issue #3): the R209/R212 divider on ILIM_HIZ may clamp
 * input current below the IINDPM register unless EN_EXT_ILIM (REG14[1]) is
 * cleared. Ask for 900 mA, measure real IBUS with the pin limit active, then
 * with it disabled. Restores IINDPM and REG14 after. Only discriminates when
 * total input demand exceeds the suspected ~600 mA clamp, so it is also called
 * late in the modem test where the modem + WiFi system load stacks on top of
 * the charge current. Draws up to 900 mA from the input for a few seconds -
 * fine on a charger, over-spec on a 500 mA port. */
static void bq_ilim_probe(const char *tag)
{
    uint8_t cs[2];
    if (!i2c_read_reg(ADDR_BQ25792, 0x1B, cs, 2) || !(cs[0] & 0x01)) return;

    uint8_t save_iindpm[2] = {0, 0}, save_r14 = 0;
    i2c_read_reg(ADDR_BQ25792, 0x06, save_iindpm, 2);
    i2c_read_reg(ADDR_BQ25792, 0x14, &save_r14, 1);

    i2c_write_reg(ADDR_BQ25792, 0x06, 0);
    i2c_write_reg(ADDR_BQ25792, 0x07, 90);          /* IINDPM = 900 mA */
    delay(2500);

    for (int phase = 0; phase < 2; phase++) {
        if (phase == 1) {
            i2c_write_reg(ADDR_BQ25792, 0x14, save_r14 & ~0x02);   /* EN_EXT_ILIM = 0 */
            delay(2000);
        }
        uint8_t b[2];
        int16_t ibus = 0, ibat = 0;
        uint16_t vbat = 0;
        if (i2c_read_reg(ADDR_BQ25792, 0x31, b, 2)) ibus = (int16_t)((b[0] << 8) | b[1]);
        if (i2c_read_reg(ADDR_BQ25792, 0x33, b, 2)) ibat = (int16_t)((b[0] << 8) | b[1]);
        if (i2c_read_reg(ADDR_BQ25792, 0x3B, b, 2)) vbat = ((uint16_t)b[0] << 8) | b[1];
        i2c_read_reg(ADDR_BQ25792, 0x1B, cs, 2);
        TEST_INFO(phase == 0 ? "ILIM probe A" : "ILIM probe B",
                  "[%s] IINDPM=900 EXT_ILIM=%s: IBUS=%d mA IBAT=%+d mA VBAT=%u mV IINDPM_STAT=%d",
                  tag, phase == 0 ? "on " : "off", ibus, ibat, vbat, (cs[0] & 0x80) ? 1 : 0);
    }

    i2c_write_reg(ADDR_BQ25792, 0x14, save_r14);
    i2c_write_reg(ADDR_BQ25792, 0x06, save_iindpm[0]);
    i2c_write_reg(ADDR_BQ25792, 0x07, save_iindpm[1]);
}

static uint16_t bq_rd16(uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    i2c_read_reg(ADDR_BQ25792, reg, b, 2);
    return ((uint16_t)b[0] << 8) | b[1];
}

/* Solar / VIN input check + panel-agnostic MPPT sweep.
 * 1. Hold both input FETs off so VAC2 shows the panel's true open-circuit
 *    voltage (Voc) - no prior knowledge of the panel needed.
 * 2. Route the charger to the solar input (ACFET2) and see if VBUS comes up.
 *    A weak panel SAGS under the qualification load (INFO, not a fault); a
 *    dead FET path leaves the panel voltage untouched (FAIL).
 * 3. Sweep VINDPM across 60-92 % of Voc measuring real input power; the peak
 *    is the panel's MPP at current illumination. (Fractional-Voc says Vmp is
 *    typically ~76 % of Voc for silicon - the sweep verifies empirically.)
 * Restores charger input routing and limits afterwards. */
static void test_solar_mppt(void)
{
    uint8_t r13o = 0, r14o = 0, iindpm_o[2], vindpm_o = 0;
    i2c_read_reg(ADDR_BQ25792, 0x13, &r13o, 1);
    i2c_read_reg(ADDR_BQ25792, 0x14, &r14o, 1);
    i2c_read_reg(ADDR_BQ25792, 0x06, iindpm_o, 2);
    i2c_read_reg(ADDR_BQ25792, 0x05, &vindpm_o, 1);

    /* Voc: both ACFETs off, panel unloaded */
    i2c_write_reg(ADDR_BQ25792, 0x13, r13o & ~0xC0);
    delay(400);
    uint16_t voc = bq_rd16(0x39);
    if (voc < 1000) {
        i2c_write_reg(ADDR_BQ25792, 0x13, r13o);
        TEST_SKIP("Solar input", "no source on VIN (VAC2=%u mV) - feed a panel / 5-20 V to test", voc);
        return;
    }
    TEST_INFO("Solar Voc", "%u mV open-circuit (panel unloaded)", voc);
    if (voc < 3800) {
        i2c_write_reg(ADDR_BQ25792, 0x13, r13o);
        TEST_INFO("Solar input", "Voc below the charger's ~3.6 V adapter-present threshold - too little light (or wiring/polarity); cannot exercise the path");
        return;
    }

    /* room to draw: input limit 2 A, pin-clamp off, VINDPM at floor for now */
    i2c_write_reg(ADDR_BQ25792, 0x14, r14o & ~0x02);   /* EN_EXT_ILIM off */
    i2c_write_reg(ADDR_BQ25792, 0x06, 0);
    i2c_write_reg(ADDR_BQ25792, 0x07, 200);            /* IINDPM = 2000 mA */
    i2c_write_reg(ADDR_BQ25792, 0x05, 36);             /* VINDPM = 3.6 V floor */

    /* solar input only: ACFET1 off, ACFET2 on */
    i2c_write_reg(ADDR_BQ25792, 0x13, (r13o & ~0x40) | 0x80);
    delay(1500);

    uint16_t vbus = bq_rd16(0x35);
    uint8_t r13v = 0;
    i2c_read_reg(ADDR_BQ25792, 0x13, &r13v, 1);
    if (vbus > voc + 300) {
        /* VBUS above the panel's own Voc can only come from another input -
         * the BQ fell back to USB despite our routing. Don't measure that. */
        TEST_INFO("Solar input", "VBUS=%u mV exceeds panel Voc=%u mV - another input is feeding the charger (EN_ACDRV2 %s); result invalid, retry with stronger light",
                  vbus, voc, (r13v & 0x80) ? "still set" : "auto-cleared");
    } else if (vbus < 3000) {
        uint16_t vac2_loaded = bq_rd16(0x39);
        if (vac2_loaded + 500 < voc)
            TEST_INFO("Solar input", "panel sags %u -> %u mV under load - source too weak to qualify (needs more light); path itself not disproven",
                      voc, vac2_loaded);
        else
            TEST_FAIL("Solar input", "VAC2 steady at %u mV but VBUS=%u mV - ACFET2 path dead (check Q202/Q203/R213/D204)",
                      voc, vbus);
    } else {
        uint8_t s[2] = {0, 0};
        i2c_read_reg(ADDR_BQ25792, 0x1B, s, 2);
        TEST_PASS("Solar input", "panel powers VBUS: %u mV, state: %s",
                  vbus, chg_names[(s[1] >> 5) & 0x07]);

        /* MPPT sweep: highest power wins */
        uint16_t best_v = 0, best_vbus = 0;
        int16_t  best_i = 0;
        int32_t  best_p = -1;
        for (int pct = 60; pct <= 92; pct += 4) {
            uint16_t vin = (uint16_t)((uint32_t)voc * pct / 100);
            if (vin < 3600) vin = 3600;
            i2c_write_reg(ADDR_BQ25792, 0x05, (uint8_t)(vin / 100));
            delay(400);
            uint16_t v = bq_rd16(0x35);
            int16_t  i = (int16_t)bq_rd16(0x31);
            int32_t  p = (int32_t)v * i / 1000;   /* mW */
            TEST_INFO("MPPT sweep", "VINDPM %u%% of Voc (%u mV): VBUS=%u mV IBUS=%d mA P=%ld mW",
                      pct, vin, v, i, (long)p);
            if (p > best_p) { best_p = p; best_v = vin; best_vbus = v; best_i = i; }
        }
        TEST_PASS("Solar MPPT", "max %ld mW at VINDPM=%u mV (%lu%% of Voc) - IBUS=%d mA",
                  (long)best_p, best_v, (unsigned long)((uint32_t)best_v * 100 / voc), best_i);
        int16_t ibat = (int16_t)bq_rd16(0x33);
        if (ibat >= 0 && ibat < 200)
            TEST_INFO("Solar MPPT", "note: battery draw only %+d mA (nearly full?) - real MPP may be higher than measured", ibat);
    }

    /* restore */
    i2c_write_reg(ADDR_BQ25792, 0x05, vindpm_o);
    i2c_write_reg(ADDR_BQ25792, 0x06, iindpm_o[0]);
    i2c_write_reg(ADDR_BQ25792, 0x07, iindpm_o[1]);
    i2c_write_reg(ADDR_BQ25792, 0x14, r14o);
    i2c_write_reg(ADDR_BQ25792, 0x13, r13o | 0x40);    /* USB gate back on */
}

static void bq_verbose_dump(void)
{
    uint8_t r[7];
    Serial.println();
    Serial.println("  --- BQ25792 verbose status dump ---");
    Serial.flush();

    if (i2c_read_reg(ADDR_BQ25792, 0x1B, r, 5)) {
        Serial.printf("  REG1B=0x%02X  IINDPM=%d VINDPM=%d WD_EXP=%d PG=%d "
                      "VBUS_PRES=%d AC2=%d AC1=%d BC12_DONE=%d\n", r[0],
                      !!(r[0]&0x80), !!(r[0]&0x40), !!(r[0]&0x20), !!(r[0]&0x10),
                      !!(r[0]&0x08), !!(r[0]&0x04), !!(r[0]&0x02), !!(r[0]&0x01));
        static const char *vbus_kind[16] = {
            "no input","USB SDP (500mA)","USB CDP","USB DCP",
            "HVDCP","unknown","non-std","OTG",
            "not-qual adapter","reserved","reserved","reserved",
            "VBAT backup","reserved","reserved","reserved" };
        Serial.printf("  REG1C=0x%02X  CHG_STAT=%s  VBUS_STAT=%s\n",
                      r[1], chg_names[(r[1]>>5)&7], vbus_kind[(r[1]>>1)&0xF]);
        static const char *ico_st[4] = { "disabled","optimizing","done","reserved" };
        Serial.printf("  REG1D=0x%02X  ICO_STAT=%s  TREG=%d  DPDM=%d  VBAT_PRESENT=%d\n",
                      r[2], ico_st[(r[2]>>6)&3], !!(r[2]&0x20), !!(r[2]&0x10), !!(r[2]&0x01));
        Serial.printf("  REG1E=0x%02X  ACRB1=%d ACRB2=%d ADC_DONE=%d VSYS=%d "
                      "CHG_TMR=%d TRICHG_TMR=%d PRECHG_TMR=%d\n", r[3],
                      !!(r[3]&0x80), !!(r[3]&0x40), !!(r[3]&0x20), !!(r[3]&0x10),
                      !!(r[3]&0x08), !!(r[3]&0x04), !!(r[3]&0x02));
        Serial.printf("  REG1F=0x%02X  VBAT_OTG_LOW=%d  TS: %s%s%s%s%s\n", r[4],
                      !!(r[4]&0x10),
                      (r[4]&0x08)?"COLD ":"", (r[4]&0x04)?"COOL ":"",
                      (r[4]&0x02)?"WARM ":"", (r[4]&0x01)?"HOT ":"",
                      ((r[4]&0x0F)==0)?"normal":"");
    }
    if (i2c_read_reg(ADDR_BQ25792, 0x01, r, 7)) {
        uint16_t vreg   = ((((uint16_t)r[0] & 0x07) << 8) | r[1]) * 10;
        uint16_t ichg   = ((((uint16_t)r[2] & 0x01) << 8) | r[3]) * 10;
        uint16_t vindpm = r[5] * 100;
        uint16_t iindpm = r[6] * 10;
        Serial.printf("  CONFIG  VREG=%u mV  ICHG=%u mA  VINDPM=%u mV  IINDPM=%u mA\n",
                      vreg, ichg, vindpm, iindpm);
    }
    if (i2c_read_reg(ADDR_BQ25792, 0x0F, r, 6)) {
        Serial.printf("  CTRL    REG0F=0x%02X (EN_CHG=%d EN_HIZ=%d EN_TERM=%d)  "
                      "REG10=0x%02X  REG11=0x%02X  REG12=0x%02X  REG13=0x%02X  REG14=0x%02X\n",
                      r[0], !!(r[0]&0x20), !!(r[0]&0x04), !!(r[0]&0x02),
                      r[1], r[2], r[3], r[4], r[5]);
    }
    Serial.println("  -----------------------------------");
    Serial.flush();
}

static void test_bq25792(void)
{
    uint8_t v = 0;
    if (!i2c_read_reg(ADDR_BQ25792, 0x48, &v, 1)) {
        TEST_FAIL("BQ25792 ID", "register read failed");
        return;
    }
    uint8_t pn = (v >> 3) & 0x07;
    if (pn != 0b001) {
        TEST_FAIL("BQ25792 ID", "REG48=0x%02X PN=0b%u%u%u (expected 0b001)",
                  v, (pn>>2)&1, (pn>>1)&1, pn&1);
        return;
    }
    TEST_PASS("BQ25792 ID", "REG48=0x%02X (PN=001 = BQ25792, REV=%u)", v, v & 0x07);

    /* Enable ADC, read VBAT and VBUS */
    if (i2c_write_reg(ADDR_BQ25792, 0x2E, 0x80)) {
        delay(150);
        uint8_t vbat[2];
        if (i2c_read_reg(ADDR_BQ25792, 0x3B, vbat, 2)) {
            uint16_t mv = ((uint16_t)vbat[0] << 8) | vbat[1];
            TEST_PASS("BQ25792 VBAT", "%u mV", mv);
        }
        uint8_t vbus[2];
        if (i2c_read_reg(ADDR_BQ25792, 0x35, vbus, 2)) {
            uint16_t mv = ((uint16_t)vbus[0] << 8) | vbus[1];
            TEST_INFO("BQ25792 VBUS", "%u mV (USB / charger input)", mv);
        }
        /* VAC1/VAC2 sense the adapter side BEFORE the ACFETs, so these tell us
         * whether 5 V physically arrives at the charger, independent of gate state. */
        uint8_t vac[2];
        if (i2c_read_reg(ADDR_BQ25792, 0x37, vac, 2)) {
            uint16_t mv = ((uint16_t)vac[0] << 8) | vac[1];
            TEST_INFO("BQ25792 VAC1", "%u mV (USB side, pre-ACFET1)", mv);
        }
        if (i2c_read_reg(ADDR_BQ25792, 0x39, vac, 2)) {
            uint16_t mv = ((uint16_t)vac[0] << 8) | vac[1];
            TEST_INFO("BQ25792 VAC2", "%u mV (solar side, pre-ACFET2)", mv);
        }
    }

    /* Charge state + nudge if not charging */
    uint8_t cs[2];
    if (i2c_read_reg(ADDR_BQ25792, 0x1B, cs, 2)) {
        uint8_t st = (cs[1] >> 5) & 0x07;
        TEST_INFO("BQ25792 chg",  "state: %s (REG1B=0x%02X REG1C=0x%02X)",
                  chg_names[st], cs[0], cs[1]);

        bool not_charging = ((cs[1] >> 5) & 0x07) == 0;
        bool vbus_present = (cs[0] & 0x01) != 0;   /* VBUS_PRESENT is REG1B[0]; [3] is PG */
        if (not_charging) {
            if (!vbus_present) {
                /* The USB input sits behind ACFET1, which is off at POR, so the
                 * charger sees no input until the gate is driven. EN_ACDRV1 is
                 * REG13[6] per SLUSDG1D -- NOT REG12[3], which is WKUP_DLY. */
                uint8_t r13 = 0;
                i2c_read_reg(ADDR_BQ25792, 0x13, &r13, 1);
                i2c_write_reg(ADDR_BQ25792, 0x13, r13 | 0x40);  /* EN_ACDRV1 */
                TEST_INFO("BQ25792 nudge", "EN_ACDRV1=1 (drive ACFET1: USB -> VBUS)");
                delay(500);
                /* The BQ auto-clears EN_ACDRV if the FET pair failed POR detection
                 * or VACx is absent -- read back to see which side rejected us. */
                uint8_t r13b = 0, r1e = 0;
                i2c_read_reg(ADDR_BQ25792, 0x13, &r13b, 1);
                i2c_read_reg(ADDR_BQ25792, 0x1E, &r1e, 1);
                TEST_INFO("BQ25792 nudge", "readback REG13=0x%02X (EN_ACDRV1 %s)  REG1E=0x%02X",
                          r13b, (r13b & 0x40) ? "stuck" : "auto-cleared", r1e);
            }
            uint8_t r10 = 0, r0f = 0;
            i2c_read_reg(ADDR_BQ25792, 0x10, &r10, 1);
            i2c_write_reg(ADDR_BQ25792, 0x10, r10 & ~0x07);
            i2c_read_reg(ADDR_BQ25792, 0x0F, &r0f, 1);
            i2c_write_reg(ADDR_BQ25792, 0x0F, (r0f | 0x20) & ~0x04);
            i2c_write_reg(ADDR_BQ25792, 0x05, 0x00);
            i2c_write_reg(ADDR_BQ25792, 0x06, 44);
            i2c_write_reg(ADDR_BQ25792, 0x07, 100);
            delay(1000);
            if (i2c_read_reg(ADDR_BQ25792, 0x1B, cs, 2)) {
                st = (cs[1] >> 5) & 0x07;
                if (st != 0) TEST_PASS("BQ25792 nudge", "after nudge: %s", chg_names[st]);
                else         TEST_INFO("BQ25792 nudge", "still not charging - check NTC, VAC1/VBUS routing");
            }

            /* Qualification watch: adapter present at VAC1 but VBUS dead means
             * either the ACFET path never conducts (bad joint at Q201/Q206/
             * R214/D203) or the source folds under the BQ's qualification
             * load. Force re-detection via an EN_HIZ pulse, then poll the
             * comparator status fast (~1 ms) and the ADC occasionally; a
             * folding source dips VAC1 / blips VBUS, a dead FET path shows a
             * rock-steady VAC1 with VBUS never twitching. */
            uint8_t vv[2];
            uint16_t vac1_now = 0;
            if (i2c_read_reg(ADDR_BQ25792, 0x37, vv, 2))
                vac1_now = ((uint16_t)vv[0] << 8) | vv[1];
            if (st == 0 && vac1_now > 4000) {
                uint8_t r0f2 = 0;
                i2c_read_reg(ADDR_BQ25792, 0x0F, &r0f2, 1);
                i2c_write_reg(ADDR_BQ25792, 0x0F, r0f2 | 0x04);   /* EN_HIZ=1 */
                delay(100);
                i2c_write_reg(ADDR_BQ25792, 0x0F, r0f2 & ~0x04);  /* EN_HIZ=0: re-detect */

                bool saw_pres = false;
                uint16_t stat_mask = 0, vbus_max = 0, vac1_min = 65535;
                unsigned long t0 = millis(), last_adc = 0;
                while (millis() - t0 < 2500) {
                    uint8_t s[2];
                    if (i2c_read_reg(ADDR_BQ25792, 0x1B, s, 2)) {
                        if (s[0] & 0x01) saw_pres = true;
                        stat_mask |= (uint16_t)1 << ((s[1] >> 1) & 0x0F);
                    }
                    if (millis() - last_adc >= 50) {
                        last_adc = millis();
                        if (i2c_read_reg(ADDR_BQ25792, 0x35, s, 2)) {
                            uint16_t v = ((uint16_t)s[0] << 8) | s[1];
                            if (v > vbus_max) vbus_max = v;
                        }
                        if (i2c_read_reg(ADDR_BQ25792, 0x37, s, 2)) {
                            uint16_t v = ((uint16_t)s[0] << 8) | s[1];
                            if (v && v < vac1_min) vac1_min = v;
                        }
                    }
                }
                TEST_INFO("BQ qual watch", "2.5 s after HIZ pulse: VBUS_PRESENT %s, VBUS_STAT mask=0x%03X, VBUSmax=%u mV, VAC1min=%u mV",
                          saw_pres ? "seen" : "never", stat_mask, vbus_max, vac1_min);
                TEST_INFO("BQ qual watch", "%s",
                          (vbus_max > 4000) ? "VBUS rose: FETs conduct - source folds under load (weak supply?)"
                        : (vac1_min < 4200 && vac1_min != 65535) ? "VAC1 sags: source folding even before the FETs"
                        : "VAC1 steady, VBUS never rose: ACFET path dead - check Q201/Q206/R214/D203 joints");
            }
        }
    }

    /* Solar / VIN input + panel-agnostic MPPT sweep (SKIPs with nothing on
     * VIN). Mind the SWAPPED +/- silkscreen, errata #1 - the true negative
     * terminal is the one that beeps to GND. */
    test_solar_mppt();

    bq_ilim_probe("bq idle");

    bq_verbose_dump();

    /* Faults */
    uint8_t f[2];
    if (i2c_read_reg(ADDR_BQ25792, 0x20, f, 2)) {
        if (f[0] == 0 && f[1] == 0) {
            TEST_PASS("BQ25792 fault", "no faults (REG20=REG21=0x00)");
        } else {
            static const char *fn0[] = { "VAC1_OVP","VAC2_OVP","CONV_OCP","IBAT_OCP",
                                         "IBUS_OCP","VBAT_OVP","VBUS_OVP","IBAT_REG" };
            static const char *fn1[] = { "TS_COLD","TS_COOL","TS_WARM","TS_HOT",
                                         "OTG_UVP","OTG_OVP","VSYS_OVP","VSYS_SHORT" };
            char buf[128] = {0};
            for (int i = 0; i < 8; i++) if (f[0] & (1 << i)) { strcat(buf, fn0[i]); strcat(buf, " "); }
            for (int i = 0; i < 8; i++) if (f[1] & (1 << i)) { strcat(buf, fn1[i]); strcat(buf, " "); }
            TEST_FAIL("BQ25792 fault", "REG20=0x%02X REG21=0x%02X -> %s", f[0], f[1], buf);
        }
    }
}

/* ===================== SC7A20 ===================== */
static void test_sc7a20(void)
{
    if (s_sc7a20_addr == 0) { TEST_FAIL("SC7A20 ID", "not seen on bus"); return; }
    uint8_t who = 0;
    if (!i2c_read_reg(s_sc7a20_addr, 0x0F, &who, 1)) {
        TEST_FAIL("SC7A20 ID", "WHO_AM_I read failed at 0x%02X", s_sc7a20_addr);
        return;
    }
    if (who != 0x11) { TEST_FAIL("SC7A20 ID", "WHO_AM_I=0x%02X (expected 0x11)", who); return; }
    TEST_PASS("SC7A20 ID", "WHO_AM_I=0x11 at 0x%02X", s_sc7a20_addr);

    /* CTRL_REG1 = 0x57: X/Y/Z on, normal mode, 100 Hz */
    if (!i2c_write_reg(s_sc7a20_addr, 0x20, 0x57)) {
        TEST_FAIL("SC7A20 read", "CTRL_REG1 write failed"); return;
    }
    delay(20);
    uint8_t xyz[6];
    if (!i2c_read_reg(s_sc7a20_addr, 0x28 | 0x80, xyz, 6)) {
        TEST_FAIL("SC7A20 read", "OUT_X..Z read failed"); return;
    }
    int16_t rx = (int16_t)((xyz[1] << 8) | xyz[0]);
    int16_t ry = (int16_t)((xyz[3] << 8) | xyz[2]);
    int16_t rz = (int16_t)((xyz[5] << 8) | xyz[4]);
    int x_mg = (rx >> 6) * 4;
    int y_mg = (ry >> 6) * 4;
    int z_mg = (rz >> 6) * 4;
    TEST_PASS("SC7A20 read", "X=%+5d Y=%+5d Z=%+5d mg (Z near +/-1000 mg if board flat)",
              x_mg, y_mg, z_mg);
}

/* ===================== LTR-303ALS ===================== */
static void test_ltr303(void)
{
    uint8_t pid = 0;
    if (!i2c_read_reg(ADDR_LTR303, 0x86, &pid, 1)) {
        TEST_FAIL("LTR-303ALS ID", "PART_ID read failed at 0x%02X", ADDR_LTR303); return;
    }
    if ((pid & 0xF0) != 0xA0) {
        TEST_FAIL("LTR-303ALS ID", "PART_ID=0x%02X (expected upper nibble 0xA)", pid); return;
    }
    TEST_PASS("LTR-303ALS ID", "PART_ID=0x%02X (part=0xA, rev=0x%X)", pid, pid & 0x0F);

    if (!i2c_write_reg(ADDR_LTR303, 0x80, 0x01)) {
        TEST_FAIL("LTR-303ALS read", "ALS_CONTR write failed"); return;
    }
    delay(150);
    uint8_t d[4];
    if (!i2c_read_reg(ADDR_LTR303, 0x88, d, 4)) {
        TEST_FAIL("LTR-303ALS read", "ALS_DATA read failed"); return;
    }
    uint16_t ch1 = ((uint16_t)d[1] << 8) | d[0];
    uint16_t ch0 = ((uint16_t)d[3] << 8) | d[2];
    TEST_PASS("LTR-303ALS read", "CH0(vis+IR)=%5u  CH1(IR)=%5u  (cover sensor to see drop)",
              ch0, ch1);
}

/* ===================== MS5837-02BA barometer (SENSOR header) ============= */
static void test_ms5837(void)
{
    MS5837 baro;
    if (!baro.isPresent()) {
        TEST_SKIP("MS5837", "no device at 0x%02X (not fitted on the I2C header)", ADDR_MS5837);
        return;
    }
    if (!baro.begin()) {
        TEST_FAIL("MS5837 PROM", "reset/PROM read failed or CRC-4 mismatch");
        return;
    }
    TEST_PASS("MS5837 PROM", "CRC-4 ok, C1..C6 = %u %u %u %u %u %u",
              baro.coefficient(1), baro.coefficient(2), baro.coefficient(3),
              baro.coefficient(4), baro.coefficient(5), baro.coefficient(6));

    float mbar = 0, degC = 0;
    if (!baro.read(mbar, degC)) {
        TEST_FAIL("MS5837 read", "conversion failed");
        return;
    }
    /* Sanity: the 02BA spans 300-1200 mbar; ~950-1000 mbar at Zurich altitude. */
    if (mbar > 300.0f && mbar < 1200.0f && degC > -20.0f && degC < 60.0f)
        TEST_PASS("MS5837 read", "%.2f mbar, %.2f C (~%.0f m ASL)",
                  mbar, degC, MS5837::altitudeM(mbar));
    else
        TEST_FAIL("MS5837 read", "implausible: %.2f mbar, %.2f C", mbar, degC);
}

/* ===================== SPI flash ===================== */
static SPIClass &spi = SPI;

static void spi_init(void)
{
    spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
    pinMode(PIN_SPI_CS_FLASH, OUTPUT);
    digitalWrite(PIN_SPI_CS_FLASH, HIGH);
    if (PIN_SPI_CS_PERIPH != PIN_SPI_CLK) {
        pinMode(PIN_SPI_CS_PERIPH, OUTPUT);
        digitalWrite(PIN_SPI_CS_PERIPH, HIGH);
    }
    TEST_PASS("SPI init", "SPI on MOSI=%d MISO=%d CLK=%d @ %d Hz",
              PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK, SPI_FREQ_HZ);
}

static void test_gd25q_flash(void)
{
    uint8_t mfg, type, cap;
    /* Release from deep power-down first (0xAB): a chip left in DPD by previous
     * firmware ignores every other command - and DPD survives resets for as long
     * as the always-on 3V3 rail is up, i.e. until the battery comes off. */
    spi.beginTransaction(SPISettings(SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS_FLASH, LOW);
    spi.transfer(0xAB);
    digitalWrite(PIN_SPI_CS_FLASH, HIGH);
    spi.endTransaction();
    delayMicroseconds(50);   /* tRES1: chip needs ~20-30 us to wake */

    spi.beginTransaction(SPISettings(SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS_FLASH, LOW);
    spi.transfer(0x9F);
    mfg  = spi.transfer(0);
    type = spi.transfer(0);
    cap  = spi.transfer(0);
    digitalWrite(PIN_SPI_CS_FLASH, HIGH);
    spi.endTransaction();

    if (mfg == 0xFF || mfg == 0x00) {
        TEST_FAIL("SPI flash", "JEDEC=0x%02X 0x%02X 0x%02X - bus floating? check MISO/CS/power",
                  mfg, type, cap);
        /* Localize the fault: with CS high (chip must not drive SO), fight the
         * line with the ESP's weak pulls. A healthy released line follows the
         * pull both ways; a solder bridge to 3V3 (SO pin 2 is next to WP#
         * pin 3, which is tied to 3V3) wins against the pulldown. */
        pinMode(PIN_SPI_MISO, INPUT_PULLDOWN);
        delay(2);
        int with_pd = digitalRead(PIN_SPI_MISO);
        pinMode(PIN_SPI_MISO, INPUT_PULLUP);
        delay(2);
        int with_pu = digitalRead(PIN_SPI_MISO);
        if (with_pd == HIGH)
            TEST_INFO("SPI MISO diag", "line HIGH even against pulldown - hard short to 3V3 (SO-WP# bridge at U602 pins 2-3?)");
        else if (with_pu == LOW)
            TEST_INFO("SPI MISO diag", "line LOW even against pullup - hard short to GND (SO-GND bridge at U602 pins 2->4 side?)");
        else
            TEST_INFO("SPI MISO diag", "line follows pulls (floating) - SO open joint, or chip not selected/powered (check CS pin 1, VCC pin 8)");
        /* pinMode() above detached MISO from the SPI matrix, and SPIClass::begin()
         * early-returns when already initialized - a bare begin() leaves the
         * peripheral reading a disconnected pin (always 0). Full end+begin. */
        spi.end();
        spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        /* ESP-side self-check: with CS held HIGH (chip deselected), clock a
         * pattern out MOSI and read MISO. With a jumper across the SPI header's
         * SI and SO pins, the pattern echoes back - proving this unit's ESP
         * drivers and the PCB traces end-to-end. Without a jumper this prints
         * the floating-line noise, which is also informative. */
        uint8_t echo[4];
        const uint8_t pat[4] = { 0xA5, 0x5A, 0x0F, 0xF0 };
        spi.beginTransaction(SPISettings(SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
        for (int i = 0; i < 4; i++) echo[i] = spi.transfer(pat[i]);
        spi.endTransaction();
        bool match = memcmp(echo, pat, 4) == 0;
        TEST_INFO("SPI echo diag", "sent A5 5A 0F F0, got %02X %02X %02X %02X -> %s",
                  echo[0], echo[1], echo[2], echo[3],
                  match ? "ECHO OK: MOSI+MISO drivers and traces proven"
                        : "no echo (expected without an SI-SO jumper at the SPI header)");

        /* Line-follow checks for the two signals the echo can't see: with the
         * jumper moved to CS-SO (or SCLK-SO), MISO must mirror that line as we
         * toggle it. digitalRead works on MISO even while SPI-attached - the
         * GPIO input register always reflects the pad. */
        /* NOTE: the SPI header's CS pin is CS_periferal (GPIO9), NOT the flash
         * CS (GPIO8) - the flash CS net has no header access. So the jumper
         * test exercises GPIO9; the flash CS gets a slow meter-probe toggle
         * below instead. */
        static const int lv[4] = { HIGH, LOW, HIGH, LOW };
        bool cs_follow = true;
        for (int i = 0; i < 4; i++) {
            digitalWrite(PIN_SPI_CS_PERIPH, lv[i]);
            delayMicroseconds(100);
            if (digitalRead(PIN_SPI_MISO) != lv[i]) cs_follow = false;
        }
        digitalWrite(PIN_SPI_CS_PERIPH, HIGH);

        pinMode(PIN_SPI_CLK, OUTPUT);           /* detaches SCLK from SPI; restored below */
        bool clk_follow = true;
        for (int i = 0; i < 4; i++) {
            digitalWrite(PIN_SPI_CLK, lv[i]);
            delayMicroseconds(100);
            if (digitalRead(PIN_SPI_MISO) != lv[i]) clk_follow = false;
        }
        spi.end();
        spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);

        TEST_INFO("SPI CS diag",   "MISO %s CS_periph/GPIO9 toggles (header CS-SO jumper; header CS is NOT flash CS)",
                  cs_follow ? "FOLLOWS" : "does not follow");
        TEST_INFO("SPI SCLK diag", "MISO %s SCLK toggles (SCLK-SO jumper -> follow = SCLK driver+trace OK)",
                  clk_follow ? "FOLLOWS" : "does not follow");

        /* Flash CS (GPIO8) has no header pin, so check it meterlessly by reading
         * back its own pad. (a) Attachment: with the internal ~45k pulldown, an
         * attached pad loses the divider against R655's 10k pullup and reads
         * HIGH; a pad orphaned by an open module joint reads LOW. (b) Drive:
         * output LOW must win against the pullup. */
        pinMode(PIN_SPI_CS_FLASH, INPUT_PULLDOWN);
        delay(2);
        bool cs_attached = digitalRead(PIN_SPI_CS_FLASH) == HIGH;
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, LOW);
        delayMicroseconds(50);
        bool cs_sinks = digitalRead(PIN_SPI_CS_FLASH) == LOW;
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        TEST_INFO("Flash CS diag", "pad %s the CS net (R655 pullup %s vs internal pulldown), driver %s sink LOW",
                  cs_attached ? "ATTACHED to" : "DETACHED from",
                  cs_attached ? "visible" : "not visible",
                  cs_sinks ? "CAN" : "CANNOT");

        /* Same sniff for CS_periph (GPIO9, external 10k R699): IO8/IO9/IO18 are
         * consecutive MINI-1 castellations (pins 22/23/24), so a second detached
         * pad points at a cracked joint strip on that module edge. */
        pinMode(PIN_SPI_CS_PERIPH, INPUT_PULLDOWN);
        delay(2);
        bool cs2_attached = digitalRead(PIN_SPI_CS_PERIPH) == HIGH;
        pinMode(PIN_SPI_CS_PERIPH, OUTPUT);
        digitalWrite(PIN_SPI_CS_PERIPH, HIGH);
        TEST_INFO("CS2 pad diag", "GPIO9 pad %s its net (R699 pullup %s)",
                  cs2_attached ? "ATTACHED to" : "DETACHED from",
                  cs2_attached ? "visible" : "not visible");

        /* Reverse SCLK check: with a jumper across the header's SI and SCLK
         * pins, GPIO18's pad should mirror MOSI toggles - tests the SCLK pad's
         * board connection using the already-proven MOSI as stimulus. */
        pinMode(PIN_SPI_MOSI, OUTPUT);          /* detaches MOSI from SPI; restored below */
        bool sclk_pad_follow = true;
        for (int i = 0; i < 4; i++) {
            digitalWrite(PIN_SPI_MOSI, lv[i]);
            delayMicroseconds(100);
            if (digitalRead(PIN_SPI_CLK) != lv[i]) sclk_pad_follow = false;
        }
        spi.end();
        spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        TEST_INFO("SCLK pad diag", "GPIO18 pad %s MOSI toggles (SI-SCLK jumper -> follow = SCLK pad attached)",
                  sclk_pad_follow ? "FOLLOWS" : "does not follow");

        /* Flash-CS net witness: with a wire from R655's chip-side pad to the
         * header's SO pin, MISO shows what the board-side CS net really does
         * while GPIO8 toggles. Follow = ESP drive reaches the net (module
         * joint OK); steady HIGH = net only sees its pullup, joint open. */
        TEST_INFO("FlashCS follow", "hold wire U602 pin 1 <-> header SO now; sampling starts in 3 s, runs 4 s");
        delay(3000);
        int csf_match = 0, csf_hi = 0, csf_n = 16;
        for (int i = 0; i < csf_n; i++) {
            int want = (i & 1) ? LOW : HIGH;
            digitalWrite(PIN_SPI_CS_FLASH, want);
            delay(250);
            int rd = digitalRead(PIN_SPI_MISO);
            if (rd == want) csf_match++;
            if (rd == HIGH) csf_hi++;
        }
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        TEST_INFO("FlashCS follow", "%d/%d samples follow GPIO8, %d/%d read HIGH -> %s",
                  csf_match, csf_n, csf_hi, csf_n,
                  (csf_match >= 14) ? "DRIVE REACHES NET - module joint OK, rethink"
                : (csf_hi >= 14)    ? "steady HIGH: net at pullup only - JOINT OPEN confirmed"
                : (csf_hi <= 2)     ? "steady LOW: wire touching GND (shield?) - reposition and rerun"
                                    : "mixed: unstable contact - rerun");

        /* Open-pad vs net-clamped-low discriminator: against only the internal
         * ~45k pullup, an OPEN pad floats HIGH; a net held down by a shorted
         * input structure in a dead chip stays LOW. */
        pinMode(PIN_SPI_CS_FLASH, INPUT_PULLUP);
        delay(2);
        int cs_pu = digitalRead(PIN_SPI_CS_FLASH);
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        pinMode(PIN_SPI_CLK, INPUT_PULLUP);
        delay(2);
        int clk_pu = digitalRead(PIN_SPI_CLK);
        pinMode(PIN_SPI_CLK, INPUT_PULLDOWN);
        delay(2);
        int clk_pd = digitalRead(PIN_SPI_CLK);
        spi.end();
        spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
        pinMode(PIN_SPI_CS_FLASH, OUTPUT);
        digitalWrite(PIN_SPI_CS_FLASH, HIGH);
        TEST_INFO("CS clamp diag",   "GPIO8 vs internal pullup reads %s -> %s",
                  cs_pu ? "HIGH" : "LOW",
                  cs_pu ? "pad OPEN at module joint (floats on own pullup)"
                        : "net CLAMPED LOW - dead chip shorting CS, replace U602");
        TEST_INFO("SCLK clamp diag", "GPIO18 pullup->%s pulldown->%s -> %s",
                  clk_pu ? "HIGH" : "LOW", clk_pd ? "HIGH" : "LOW",
                  (!clk_pu) ? "net CLAMPED LOW - dead chip shorting SCLK, replace U602"
                            : (clk_pd ? "stuck HIGH?" : "follows pulls (open pad or healthy hi-Z net)"));
        return;
    }
    if (mfg != 0xC8) {
        TEST_FAIL("SPI flash", "JEDEC=0x%02X 0x%02X 0x%02X - manufacturer not GigaDevice (0xC8)",
                  mfg, type, cap);
        return;
    }
    int mbits = (cap >= 0x10 && cap <= 0x20) ? (1 << (cap - 0x11)) : 0;
    TEST_PASS("SPI flash", "JEDEC=0x%02X 0x%02X 0x%02X (GD25Q%d, %d Mbit / %d MB)",
              mfg, type, cap, mbits, mbits, mbits / 8);
}

static void test_cs2_empty(void)
{
    if (PIN_SPI_CS_PERIPH == PIN_SPI_CLK) {
        TEST_SKIP("SPI CS2", "GPIO9 borrowed as SCLK (bench bypass)");
        return;
    }
    digitalWrite(PIN_SPI_CS_PERIPH, HIGH); delay(1);
    digitalWrite(PIN_SPI_CS_PERIPH, LOW);  delay(1);
    digitalWrite(PIN_SPI_CS_PERIPH, HIGH);
    TEST_SKIP("SPI CS2", "no device populated; GPIO%d toggled H-L-H", PIN_SPI_CS_PERIPH);
}

/* ===================== Interrupt pins ===================== */
static void test_interrupt_pins(void)
{
    SECTION("Interrupt / wake pins (external pullups -> idle HIGH expected)");
    pinMode(PIN_QON,        INPUT_PULLUP);
    pinMode(PIN_INT_SHARED, INPUT_PULLUP);
    pinMode(PIN_INT_BQ,     INPUT_PULLUP);
    delay(5);
    int qon = digitalRead(PIN_QON);
    int ish = digitalRead(PIN_INT_SHARED);
    int ibq = digitalRead(PIN_INT_BQ);

    if (ish == HIGH) TEST_PASS("INT_shared", "GPIO%d idle HIGH (accel + LTR303)", PIN_INT_SHARED);
    else             TEST_FAIL("INT_shared", "GPIO%d reads LOW (a sensor asserting?)", PIN_INT_SHARED);
    if (ibq == HIGH) TEST_PASS("INT_bq",     "GPIO%d idle HIGH", PIN_INT_BQ);
    else             TEST_FAIL("INT_bq",     "GPIO%d reads LOW", PIN_INT_BQ);
    /* DO NOT drive QON low - would risk BQ ship-mode */
    if (qon == HIGH) TEST_PASS("QON",        "GPIO%d idle HIGH", PIN_QON);
    else             TEST_INFO("QON",        "GPIO%d reads LOW (button held? BQ asserting?)", PIN_QON);
}

/* ===================== Modem helpers ===================== */
static void modem_pwrkey_pulse(int ms)
{
    digitalWrite(PIN_MODEM_PWRKEY, LOW);
    delay(ms);
    digitalWrite(PIN_MODEM_PWRKEY, HIGH);
}

static int wait_for_status(int target_level, int max_ms)
{
    int n = max_ms / 500;
    for (int i = 0; i < n; i++) {
        delay(500);
        if (digitalRead(PIN_MODEM_STATUS) == target_level) return (i + 1) * 500;
    }
    return -1;
}

static bool modem_send_at_expect_ok(const char *cmd, char *resp, size_t resp_sz, int timeout_ms)
{
    while (modem.available()) modem.read();
    modem.print(cmd);
    modem.print("\r\n");

    memset(resp, 0, resp_sz);
    size_t got = 0;
    unsigned long t0 = millis();
    while ((millis() - t0) < (unsigned long)timeout_ms && got < resp_sz - 1) {
        while (modem.available() && got < resp_sz - 1) {
            resp[got++] = (char)modem.read();
        }
        if (strstr(resp, "OK\r\n"))    return true;
        if (strstr(resp, "ERROR"))     return false;
        delay(10);
    }
    return false;
}

static bool modem_wait_for(const char *needle, char *resp, size_t resp_sz, int timeout_ms)
{
    memset(resp, 0, resp_sz);
    size_t got = 0;
    unsigned long t0 = millis();
    while ((millis() - t0) < (unsigned long)timeout_ms && got < resp_sz - 1) {
        while (modem.available() && got < resp_sz - 1) {
            resp[got++] = (char)modem.read();
        }
        if (strstr(resp, needle)) return true;
        delay(10);
    }
    return false;
}

/* Baud sweep: the passive UART level shifter (10k pull-ups, Q402/Q403) limits
 * rise time, so the usable rate is a hardware property worth measuring per
 * board. Escalates until a rate fails, then always restores 115200 so the
 * AT-based shutdown still works. A rate only counts as good after 10/10 clean
 * AT round-trips plus one multi-line ATI response. */
static void test_modem_baud_sweep(char *resp, size_t resp_sz)
{
    static const uint32_t rates[] = { 230400, 460800, 921600, 1843200, 3686400 };
    uint32_t good = MODEM_BAUD;   /* fastest rate proven reliable */
    uint32_t cur  = MODEM_BAUD;   /* rate the modem is actually on right now */
    char cmd[32];

    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        uint32_t rate = rates[r];
        snprintf(cmd, sizeof(cmd), "AT+IPR=%lu", (unsigned long)rate);
        if (!modem_send_at_expect_ok(cmd, resp, resp_sz, 2000)) {
            TEST_INFO("Baud sweep", "%lu: AT+IPR rejected - stopping", (unsigned long)rate);
            break;
        }
        modem.updateBaudRate(rate);
        cur = rate;
        delay(100);
        while (modem.available()) modem.read();
        /* Warm-up exchange: the first AT after an IPR switch reliably drops
         * (measured on hardware); discard it so the score reflects steady state. */
        modem_send_at_expect_ok("AT", resp, resp_sz, 300);

        int ok = 0;
        char failed_at[24] = "";
        for (int i = 0; i < 10; i++) {
            if (modem_send_at_expect_ok("AT", resp, resp_sz, 300)) {
                ok++;
            } else {
                size_t len = strlen(failed_at);
                snprintf(failed_at + len, sizeof(failed_at) - len, "%s#%d",
                         len ? "," : "", i + 1);
            }
        }
        bool ati_ok = modem_send_at_expect_ok("ATI", resp, resp_sz, 1000);

        if (ok == 10 && ati_ok) {
            TEST_PASS("Baud sweep", "%lu: 10/10 AT + ATI clean", (unsigned long)rate);
            good = rate;
        } else {
            TEST_INFO("Baud sweep", "%lu: %d/10 AT (dropped %s), ATI %s - unreliable, stopping",
                      (unsigned long)rate, ok, failed_at[0] ? failed_at : "none",
                      ati_ok ? "ok" : "failed");
            break;
        }
    }

    /* Restore 115200. The modem may be stranded on an unreliable rate; the
     * ESP->modem direction often still gets a short command through, so send
     * AT+IPR=115200 at the modem's current rate and retry a few times. */
    bool back = false;
    for (int attempt = 0; attempt < 3 && !back; attempt++) {
        modem_send_at_expect_ok("AT+IPR=115200", resp, resp_sz, 500);
        modem.updateBaudRate(115200);
        delay(100);
        while (modem.available()) modem.read();
        back = modem_send_at_expect_ok("AT", resp, resp_sz, 500);
        if (!back) {
            modem.updateBaudRate(cur);
            delay(50);
        }
    }
    if (back) {
        TEST_PASS("Baud restore", "back at 115200; fastest reliable = %lu",
                  (unsigned long)good);
    } else {
        TEST_FAIL("Baud restore", "modem not answering at 115200 - PWRKEY fallback will handle shutdown");
    }
}

/* ===================== Modem test ===================== */
static void test_modem(void)
{
    SECTION("A7672 modem (power + STATUS + AT-based shutdown)");

    pinMode(PIN_MODEM_STATUS, INPUT_PULLUP);
    TEST_INFO("STATUS init", "GPIO%d = %d (rail still off)",
              PIN_MODEM_STATUS, digitalRead(PIN_MODEM_STATUS));

    digitalWrite(PIN_MODEM_PWR_EN, HIGH);
    TEST_INFO("PWR_EN ON",   "GPIO%d -> HIGH (apply VBAT)", PIN_MODEM_PWR_EN);
    delay(200);

    TEST_INFO("PWRKEY",      "pulse LOW for %d ms (boot)", MODEM_PWRKEY_ON_MS);
    modem_pwrkey_pulse(MODEM_PWRKEY_ON_MS);

    int t_high = wait_for_status(HIGH, 10000);
    if (t_high >= 0) {
        TEST_PASS("Modem ON",  "STATUS went HIGH %d ms after PWRKEY", t_high);
    } else {
        TEST_FAIL("Modem ON",  "STATUS never went HIGH within 10 s - modem did not boot");
        digitalWrite(PIN_MODEM_PWR_EN, LOW);
        return;
    }

    modem.begin(MODEM_BAUD, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);

    char resp[256];
    bool at_ok = false;
    for (int i = 0; i < 15 && !at_ok; i++) {
        at_ok = modem_send_at_expect_ok("AT", resp, sizeof(resp), 1000);
    }
    if (!at_ok) {
        TEST_FAIL("Modem AT", "no OK from AT after 15 s polling");
    } else {
        TEST_PASS("Modem AT", "OK");

        if (modem_send_at_expect_ok("ATI", resp, sizeof(resp), 2000)) {
            for (size_t i = 0; i < strlen(resp); i++)
                if (resp[i] == '\r' || resp[i] == '\n') resp[i] = ' ';
            TEST_INFO("Modem ATI", "%s", resp);
        }

        if (modem_send_at_expect_ok("AT+CBC", resp, sizeof(resp), 2000)) {
            char raw[160] = {0};
            strncpy(raw, resp, sizeof(raw) - 1);
            for (size_t i = 0; i < strlen(raw); i++)
                if (raw[i] == '\r' || raw[i] == '\n') raw[i] = ' ';
            TEST_INFO("Modem CBC raw", "%s", raw);

            char *p = strstr(resp, "+CBC:");
            int bcs = 0, bcl = 0, mv = 0, vint = 0, vfrac = 0;
            if (p && sscanf(p, "+CBC: %d,%d,%d", &bcs, &bcl, &mv) == 3) {
                static const char *bcs_str[] =
                    { "not charging", "charging", "charged", "charge error" };
                TEST_PASS("Modem VBAT", "%d mV (%s, batt %d%%)", mv,
                          (bcs >= 0 && bcs <= 3) ? bcs_str[bcs] : "?", bcl);
            } else if (p && sscanf(p, "+CBC: %d.%d", &vint, &vfrac) == 2) {
                int mult = (vfrac < 10) ? 100 : (vfrac < 100) ? 10 : 1;
                int vmv  = vint * 1000 + vfrac * mult;
                TEST_PASS("Modem VBAT", "%d.%03dV (~%d mV) [unreliable; trust BQ25792 VBAT]",
                          vint, vfrac * mult, vmv);
            } else {
                TEST_INFO("Modem VBAT", "format unrecognised");
            }
        }

        /* SIM */
        while (modem.available()) modem.read();
        modem.print("AT+CPIN?\r\n");
        if (modem_wait_for("+CPIN:", resp, sizeof(resp), 5000)) {
            if (strstr(resp, "READY")) TEST_PASS("Modem SIM", "READY");
            else                       TEST_FAIL("Modem SIM", "not READY");
        } else {
            TEST_FAIL("Modem SIM", "no +CPIN: line within 5 s");
        }

        /* CREG poll */
        bool registered = false;
        int reg_stat = -1;
        for (int i = 0; i < 30 && !registered; i++) {
            if (modem_send_at_expect_ok("AT+CREG?", resp, sizeof(resp), 2000)) {
                char *p = strstr(resp, "+CREG:");
                int n;
                if (p && sscanf(p, "+CREG: %d,%d", &n, &reg_stat) == 2 &&
                    (reg_stat == 1 || reg_stat == 5)) {
                    registered = true; break;
                }
            }
            delay(1000);
        }
        if (!registered) {
            TEST_FAIL("Modem CREG", "not registered after 30 s (last stat=%d)", reg_stat);
        } else {
            TEST_PASS("Modem CREG", "registered (stat=%d, %s)",
                      reg_stat, reg_stat == 1 ? "home" : "roaming");

            if (modem_send_at_expect_ok("AT+CSQ", resp, sizeof(resp), 2000)) {
                char *p = strstr(resp, "+CSQ:");
                int rssi = -1, ber = -1;
                if (p && sscanf(p, "+CSQ: %d,%d", &rssi, &ber) == 2) {
                    int dbm = (rssi == 99) ? 0 : (-113 + rssi * 2);
                    TEST_INFO("Modem CSQ", "rssi=%d (%d dBm)  ber=%d", rssi, dbm, ber);
                }
            }

            char cmd[96];
            snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", MODEM_APN);
            modem_send_at_expect_ok(cmd, resp, sizeof(resp), 2000);
            bool gprs = modem_send_at_expect_ok("AT+CGACT=1,1", resp, sizeof(resp), 30000);
            if (!gprs) {
                TEST_FAIL("Modem GPRS", "AT+CGACT=1,1 failed (APN=%s)", MODEM_APN);
            } else {
                TEST_PASS("Modem GPRS", "PDP context active (APN=%s)", MODEM_APN);

                if (modem_send_at_expect_ok("AT+CDNSGIP=\"www.google.com\"",
                                            resp, sizeof(resp), 15000)) {
                    char *p = strstr(resp, "+CDNSGIP:");
                    if (p) for (size_t i = 0; i < strlen(p); i++)
                        if (p[i] == '\r' || p[i] == '\n') { p[i] = 0; break; }
                    TEST_INFO("Modem DNS", "%s", p ? p : "(no +CDNSGIP)");
                }
                if (modem_send_at_expect_ok("AT+CGPADDR=1", resp, sizeof(resp), 2000)) {
                    char *p = strstr(resp, "+CGPADDR:");
                    if (p) for (size_t i = 0; i < strlen(p); i++)
                        if (p[i] == '\r' || p[i] == '\n') { p[i] = 0; break; }
                    TEST_INFO("Modem CGPADDR", "%s", p ? p : "(no IP)");
                }

                modem_send_at_expect_ok("AT+HTTPTERM", resp, sizeof(resp), 1000);
                if (modem_send_at_expect_ok("AT+HTTPINIT", resp, sizeof(resp), 2000)) {
                    modem_send_at_expect_ok("AT+HTTPPARA=\"CID\",1",
                                            resp, sizeof(resp), 2000);
                    snprintf(cmd, sizeof(cmd),
                             "AT+HTTPPARA=\"URL\",\"%s\"", HTTP_TEST_URL);
                    modem_send_at_expect_ok(cmd, resp, sizeof(resp), 2000);

                    bool action_accepted = modem_send_at_expect_ok(
                        "AT+HTTPACTION=0", resp, sizeof(resp), 5000);
                    if (!action_accepted) {
                        for (size_t i = 0; i < strlen(resp); i++)
                            if (resp[i] == '\r' || resp[i] == '\n') resp[i] = ' ';
                        TEST_FAIL("Modem HTTP", "AT+HTTPACTION not accepted: %s", resp);
                    } else if (modem_wait_for("+HTTPACTION:", resp, sizeof(resp), 60000)) {
                        char *p = strstr(resp, "+HTTPACTION:");
                        int method = 0, status = 0, length = 0;
                        if (p && sscanf(p, "+HTTPACTION: %d,%d,%d",
                                        &method, &status, &length) == 3) {
                            if (status == 200 || status == 204) {
                                TEST_PASS("Modem HTTP", "GET %s -> %d (%d bytes)",
                                          HTTP_TEST_URL, status, length);
                            } else {
                                TEST_FAIL("Modem HTTP", "GET %s -> %d", HTTP_TEST_URL, status);
                            }
                        } else {
                            TEST_FAIL("Modem HTTP", "+HTTPACTION unparsed");
                        }
                    } else {
                        TEST_FAIL("Modem HTTP", "no +HTTPACTION within 60 s");
                    }
                    modem_send_at_expect_ok("AT+HTTPTERM", resp, sizeof(resp), 2000);
                } else {
                    TEST_FAIL("Modem HTTP", "AT+HTTPINIT failed");
                }
            }
        }

        bq_ilim_probe("modem+wifi on");

        test_modem_baud_sweep(resp, sizeof(resp));

        bool cpof_ok = modem_send_at_expect_ok("AT+CPOF", resp, sizeof(resp), 2000);
        TEST_INFO("AT+CPOF",   "%s", cpof_ok ? "OK" : "no OK / timeout");
    }

    int t_low = wait_for_status(LOW, 20000);
    if (t_low >= 0) {
        TEST_PASS("Modem OFF", "STATUS went LOW %d ms after AT+CPOF", t_low);
    } else {
        TEST_INFO("Fallback",  "long PWRKEY pulse %d ms", MODEM_PWRKEY_OFF_MS);
        modem_pwrkey_pulse(MODEM_PWRKEY_OFF_MS);
        t_low = wait_for_status(LOW, 15000);
        if (t_low >= 0) TEST_PASS("Modem OFF", "STATUS went LOW %d ms after long PWRKEY", t_low);
        else            TEST_FAIL("Modem OFF", "STATUS still HIGH - forcing rail off");
    }

    digitalWrite(PIN_MODEM_PWR_EN, LOW);
    TEST_INFO("PWR_EN OFF",  "GPIO%d -> LOW (rail off)", PIN_MODEM_PWR_EN);
}

/* ===================== WiFi + HTTP ===================== */
static void test_wifi_http(void)
{
    SECTION("WiFi STA + HTTP GET");
    TEST_INFO("WiFi conn", "joining \"%s\" ...", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_MS) {
        delay(200);
    }
    if (WiFi.status() != WL_CONNECTED) {
        TEST_FAIL("WiFi conn", "no IP within %d ms (status=%d)", WIFI_CONNECT_MS, WiFi.status());
        return;
    }
    IPAddress ip = WiFi.localIP();
    TEST_PASS("WiFi conn", "got IP %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

    HTTPClient http;
    http.setConnectTimeout(10000);
    http.setTimeout(10000);
    http.begin(HTTP_TEST_URL);
    int code = http.GET();
    http.end();
    if (code == 200 || code == 204) {
        TEST_PASS("WiFi HTTP", "GET %s -> %d", HTTP_TEST_URL, code);
    } else {
        TEST_FAIL("WiFi HTTP", "GET %s -> %d", HTTP_TEST_URL, code);
    }
}

/* ===================== setup() / loop() ===================== */
void setup(void)
{
    Serial.begin(115200);
    delay(1500);   /* USB-CDC enumeration */

    Serial.println();
    Serial.println();
    Serial.println("================================================");
    Serial.println("  Communication Module - Functionality Test (Arduino)");
    Serial.println("  Target : ESP32-C6-MINI-1");
    Serial.print  ("  Build  : "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
    Serial.println("================================================");
    Serial.flush();

    led_start();

    test_power_rails();

    SECTION("I2C init");
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    TEST_PASS("I2C init", "Wire on GPIO%d/%d @ %d Hz", PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    test_i2c_scan();

    SECTION("I2C device tests");
    test_atecc608b();
    test_bq25792();
    test_sc7a20();
    test_ltr303();
    test_ms5837();

    SECTION("SPI init");
    spi_init();
    SECTION("SPI device tests");
    test_gd25q_flash();
    test_cs2_empty();

    test_interrupt_pins();
    test_wifi_http();
    test_modem();

    Serial.println();
    Serial.println("================================================");
    Serial.printf ("  Result: %d PASS  /  %d FAIL  /  %d SKIP\n", g_pass, g_fail, g_skip);
    Serial.println("================================================");
    Serial.println();
    Serial.println("LED solid ON - tests complete.");

    delay(LED_TAIL_BLINK_MS);
    led_stop_solid_on();

    /* Replay everything in case early lines were lost on USB */
    Serial.println();
    Serial.println("================================================");
    Serial.println("  FULL REPORT (replay - in case early lines were lost on USB)");
    Serial.println("================================================");
    Serial.write((const uint8_t *)s_log_buf, s_log_len);
    Serial.printf("\n  Result: %d PASS  /  %d FAIL  /  %d SKIP\n", g_pass, g_fail, g_skip);
    Serial.println("================================================");
    Serial.flush();
}

void loop(void)
{
    delay(60000);
}
