#include "sensors.h"
#include "i2c_bus.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define XFER_MS 100

/* ---- shared small helpers ---- */

static bool dev_rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, size_t n)
{
    if (!d) return false;
    return i2c_master_transmit_receive(d, &reg, 1, buf, n, XFER_MS) == ESP_OK;
}

static bool dev_wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v)
{
    if (!d) return false;
    uint8_t tx[2] = { reg, v };
    return i2c_master_transmit(d, tx, 2, XFER_MS) == ESP_OK;
}

/* ===================== LTR-303 (0x29) ===================== */

static i2c_master_dev_handle_t s_ltr;

bool ltr303_sample(uint16_t *ch0, uint16_t *ch1)
{
    *ch0 = *ch1 = 0;
    if (!s_ltr) eco_i2c_add_tracked(&s_ltr, 0x29);

    uint8_t pid = 0;
    if (!dev_rd(s_ltr, 0x86, &pid, 1) || (pid & 0xF0) != 0xA0) return false;

    dev_wr(s_ltr, 0x80, 0x01);              /* ALS_CONTR: active */
    vTaskDelay(pdMS_TO_TICKS(150));         /* one integration (~100 ms default) */

    /* CH1 low byte read latches the pair; burst from 0x88 covers both. */
    uint8_t d[4];
    bool ok = dev_rd(s_ltr, 0x88, d, 4);
    if (ok) {
        *ch1 = ((uint16_t)d[1] << 8) | d[0];
        *ch0 = ((uint16_t)d[3] << 8) | d[2];
    }
    dev_wr(s_ltr, 0x80, 0x00);              /* standby */
    return ok;
}

/* ===================== SC7A20 (0x18 / 0x19) ===================== */

static i2c_master_dev_handle_t s_acc;

bool sc7a20_sample(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    *x_mg = *y_mg = *z_mg = 0;

    if (!s_acc) {
        i2c_master_bus_handle_t bus = eco_i2c_bus();
        if (!bus) return false;
        uint8_t addr = (i2c_master_probe(bus, 0x18, XFER_MS) == ESP_OK) ? 0x18 :
                       (i2c_master_probe(bus, 0x19, XFER_MS) == ESP_OK) ? 0x19 : 0;
        if (!addr) return false;
        eco_i2c_add_tracked(&s_acc, addr);
    }

    uint8_t who = 0;
    if (!dev_rd(s_acc, 0x0F, &who, 1) || who != 0x11) return false;

    if (!dev_wr(s_acc, 0x20, 0x57)) return false;   /* X/Y/Z on, 100 Hz */
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t d[6];
    bool ok = dev_rd(s_acc, 0x28 | 0x80, d, 6);     /* 0x80 = auto-increment */
    if (ok) {
        *x_mg = (int16_t)(((int16_t)((d[1] << 8) | d[0]) >> 6) * 4);   /* 10-bit, 4 mg/LSB */
        *y_mg = (int16_t)(((int16_t)((d[3] << 8) | d[2]) >> 6) * 4);
        *z_mg = (int16_t)(((int16_t)((d[5] << 8) | d[4]) >> 6) * 4);
    }
    dev_wr(s_acc, 0x20, 0x00);                      /* power down */
    return ok;
}

/* --- motion wake ---------------------------------------------------------
 * Register set is LIS2DH-compatible. The sequence matters: configure while the
 * interrupt is disabled, reset the high-pass filter by reading REFERENCE, then
 * enable the INT1 pin last, or the latch fires on the configuration transient
 * itself and every sleep wakes instantly. */
bool sc7a20_arm_motion(uint16_t threshold_mg)
{
    if (!s_acc) return false;

    uint8_t ths = (uint8_t)(threshold_mg / 16);     /* 16 mg/LSB at +-2 g */
    if (ths == 0) ths = 1;
    if (ths > 0x7F) ths = 0x7F;

    bool ok = true;
    ok &= dev_wr(s_acc, 0x22, 0x00);   /* CTRL_REG3: INT1 pin off while we set up */
    ok &= dev_wr(s_acc, 0x20, 0x2F);   /* CTRL_REG1: 10 Hz, low-power, X/Y/Z on */
    ok &= dev_wr(s_acc, 0x21, 0x01);   /* CTRL_REG2: high-pass filter feeds INT1
                                        * -- without it 1 g of gravity sits above
                                        * any useful threshold and the interrupt
                                        * is permanently asserted */
    ok &= dev_wr(s_acc, 0x23, 0x00);   /* CTRL_REG4: +-2 g */
    ok &= dev_wr(s_acc, 0x24, 0x08);   /* CTRL_REG5: latch INT1 (LIR_INT1) */
    ok &= dev_wr(s_acc, 0x25, 0x02);   /* CTRL_REG6: H_LACTIVE -> INT is ACTIVE LOW */
    ok &= dev_wr(s_acc, 0x32, ths);    /* INT1_THS */
    ok &= dev_wr(s_acc, 0x33, 0x01);   /* INT1_DURATION: 1 sample at 10 Hz */

    uint8_t ref = 0;
    dev_rd(s_acc, 0x26, &ref, 1);      /* REFERENCE: resets the HP filter */
    uint8_t src = 0;
    dev_rd(s_acc, 0x31, &src, 1);      /* INT1_SRC: clear any stale latch */

    ok &= dev_wr(s_acc, 0x30, 0x2A);   /* INT1_CFG: OR of X/Y/Z high events */
    ok &= dev_wr(s_acc, 0x22, 0x40);   /* CTRL_REG3: route AOI1 to the INT1 pin */
    return ok;
}

bool sc7a20_motion_fired(void)
{
    if (!s_acc) return false;
    uint8_t src = 0;
    if (!dev_rd(s_acc, 0x31, &src, 1)) return false;   /* the read clears the latch */
    return (src & 0x40) != 0;                          /* IA: an event was active */
}

/* ===================== MS5837-02BA (0x76) ===================== */

static i2c_master_dev_handle_t s_baro;

static bool ms_cmd(uint8_t c)
{
    if (!s_baro) return false;
    return i2c_master_transmit(s_baro, &c, 1, XFER_MS) == ESP_OK;
}

static bool ms_read_n(uint8_t *buf, size_t n)
{
    if (!s_baro) return false;
    return i2c_master_receive(s_baro, buf, n, XFER_MS) == ESP_OK;
}

/* CRC-4 per datasheet reference implementation; destroys the array. */
static uint8_t ms_crc4(uint16_t *prom)
{
    uint16_t rem = 0;
    prom[0] &= 0x0FFF;
    prom[7] = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (i & 1) rem ^= (uint16_t)(prom[i >> 1] & 0x00FF);
        else       rem ^= (uint16_t)(prom[i >> 1] >> 8);
        for (uint8_t bit = 8; bit > 0; bit--)
            rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000)
                                 : (uint16_t)(rem << 1);
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}

static bool ms_convert(uint8_t cmd, uint32_t *value)
{
    if (!ms_cmd(cmd)) return false;
    vTaskDelay(pdMS_TO_TICKS(20));   /* 9.04 ms typ at OSR 4096 */
    if (!ms_cmd(0x00)) return false; /* ADC read */
    uint8_t d[3];
    if (!ms_read_n(d, 3)) return false;
    *value = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    return true;
}

bool ms5837_sample(float *mbar, float *degc)
{
    *mbar = *degc = 0;
    if (!s_baro) eco_i2c_add_tracked(&s_baro, 0x76);

    if (!ms_cmd(0x1E)) return false;          /* reset */
    vTaskDelay(pdMS_TO_TICKS(20));

    uint16_t c[8];
    for (uint8_t i = 0; i < 7; i++) {         /* PROM */
        if (!ms_cmd((uint8_t)(0xA0 + (i << 1)))) return false;
        uint8_t d[2];
        if (!ms_read_n(d, 2)) return false;
        c[i] = ((uint16_t)d[0] << 8) | d[1];
    }
    c[7] = 0;
    uint16_t check[8];
    memcpy(check, c, sizeof(check));
    if (ms_crc4(check) != (uint8_t)((c[0] >> 12) & 0x0F)) return false;

    uint32_t d1 = 0, d2 = 0;
    if (!ms_convert(0x48, &d1)) return false; /* pressure, OSR 4096 */
    if (!ms_convert(0x58, &d2)) return false; /* temperature */
    if (d1 == 0 || d2 == 0) return false;

    /* First + second order compensation, 02BA scaling (datasheet). */
    int32_t dT   = (int32_t)d2 - (int32_t)c[5] * 256;
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * c[6]) / 8388608LL);
    int64_t OFF  = (int64_t)c[2] * 131072LL + ((int64_t)c[4] * dT) / 64LL;
    int64_t SENS = (int64_t)c[1] * 65536LL  + ((int64_t)c[3] * dT) / 128LL;

    int64_t Ti = 0, OFFi = 0, SENSi = 0;
    if (TEMP < 2000) {
        int64_t dt2 = (int64_t)(TEMP - 2000) * (TEMP - 2000);
        Ti    = (11 * (int64_t)dT * dT) / 34359738368LL;
        OFFi  = (31 * dt2) / 8;
        SENSi = (63 * dt2) / 32;
    }
    int32_t P = (int32_t)((((int64_t)d1 * (SENS - SENSi)) / 2097152LL - (OFF - OFFi)) / 32768LL);

    *mbar = (float)P / 100.0f;
    *degc = (float)(TEMP - (int32_t)Ti) / 100.0f;
    return true;
}
