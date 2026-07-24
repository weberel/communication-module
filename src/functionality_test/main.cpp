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
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <stdarg.h>

/* ===================== Pin map ===================== */
#define PIN_I2C_SDA          6
#define PIN_I2C_SCL          7

#define PIN_SPI_MOSI         4
#define PIN_SPI_MISO         5
#define PIN_SPI_CLK          18
#define PIN_SPI_CS_FLASH     8
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
    TEST_INFO("SENSOR_PWR",   "GPIO%d ext-sensor rail switch (populated); on-board sensors on 3V3", PIN_SENSOR_PWR);
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
    }

    /* Charge state + nudge if not charging */
    uint8_t cs[2];
    if (i2c_read_reg(ADDR_BQ25792, 0x1B, cs, 2)) {
        uint8_t st = (cs[1] >> 5) & 0x07;
        TEST_INFO("BQ25792 chg",  "state: %s (REG1B=0x%02X REG1C=0x%02X)",
                  chg_names[st], cs[0], cs[1]);

        bool not_charging = ((cs[1] >> 5) & 0x07) == 0;
        bool vbus_present = (cs[0] & 0x08) != 0;
        bool ac1_present  = (cs[0] & 0x02) != 0;
        if (not_charging && (vbus_present || ac1_present)) {
            if (ac1_present && !vbus_present) {
                uint8_t r12 = 0;
                i2c_read_reg(ADDR_BQ25792, 0x12, &r12, 1);
                i2c_write_reg(ADDR_BQ25792, 0x12, r12 | 0x08);  /* EN_ACDRV1 */
                TEST_INFO("BQ25792 nudge", "EN_ACDRV1=1 (AC1 sense -> VBUS gate)");
                delay(500);
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
        }
    }

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

/* ===================== SPI flash ===================== */
static SPIClass &spi = SPI;

static void spi_init(void)
{
    spi.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS_FLASH);
    pinMode(PIN_SPI_CS_FLASH, OUTPUT);
    digitalWrite(PIN_SPI_CS_FLASH, HIGH);
    pinMode(PIN_SPI_CS_PERIPH, OUTPUT);
    digitalWrite(PIN_SPI_CS_PERIPH, HIGH);
    TEST_PASS("SPI init", "SPI on MOSI=%d MISO=%d CLK=%d @ %d Hz",
              PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK, SPI_FREQ_HZ);
}

static void test_gd25q_flash(void)
{
    uint8_t mfg, type, cap;
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
