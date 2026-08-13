#include "uss.h"
#include "uss_link.h"
#include "config.h"
#include "i2c_bus.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uss";

/* Generous per-transaction timeout: the eUSCI slave clock-stretches while its
 * ISR runs, and may mask its I2C IRQ around the timing-critical ultrasonic
 * capture -- a stalled poll is normal, not an error. */
#define XFER_TO_MS 1000

static i2c_master_dev_handle_t s_dev;

static bool rd(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, XFER_TO_MS) == ESP_OK;
}

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, XFER_TO_MS) == ESP_OK;
}

/* Read + CRC-check the latched result block; one retry covers a read that
 * raced a (misbehaving) mid-update slave or a glitched byte on the harness. */
static bool read_result(uint8_t *blk)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!rd(USS_LINK_RESULT_OFF, blk, USS_LINK_RESULT_LEN)) return false;
        if (uss_link_crc8(blk, USS_LINK_RESULT_LEN - 1) ==
            blk[USS_LINK_RESULT_LEN - 1]) return true;
        ESP_LOGW(TAG, "result CRC mismatch (attempt %d)", attempt + 1);
    }
    return false;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

bool uss_sample(uss_result_t *out)
{
    memset(out, 0, sizeof(*out));

    if (!s_dev) {
        s_dev = eco_i2c_add(USS_LINK_ADDR7);
        if (!s_dev) return false;
    }

    /* Identify -- doubles as the absence probe (absent module = one NACK). */
    uint8_t id[2];
    if (!rd(USS_REG_WHO_AM_I, id, 2)) return false;
    if (id[0] != USS_LINK_WHOAMI || id[1] != USS_LINK_PROTO) {
        ESP_LOGW(TAG, "bad identity 0x%02x proto %u", id[0], id[1]);
        return false;
    }

    if (!wr(USS_REG_CMD, USS_CMD_MEASURE)) return false;

    /* Poll STATUS until READY. First wait is longer: the slave is doing the
     * whole capture + algorithm run, no point hammering the bus. */
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t st = 0;
    int waited = 100;
    for (;;) {
        if (!rd(USS_REG_STATUS, &st, 1)) return false;
        if ((st & USS_ST_READY) && !(st & USS_ST_BUSY)) break;
        if (waited >= USS_MEAS_TIMEOUT_MS) {
            ESP_LOGW(TAG, "measurement timeout (status 0x%02x)", st);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(USS_POLL_MS));
        waited += USS_POLL_MS;
    }

    uint8_t blk[USS_LINK_RESULT_LEN];
    if (!read_result(blk)) return false;

    /* blk[] is the register window starting at USS_REG_STATUS (0x04). */
    out->status    = blk[USS_REG_STATUS    - USS_LINK_RESULT_OFF];
    out->code      = blk[USS_REG_CODE      - USS_LINK_RESULT_OFF];
    out->seq       = le16(&blk[USS_REG_SEQ - USS_LINK_RESULT_OFF]);
    out->flow_ulpm = (int32_t)le32(&blk[USS_REG_FLOW_ULPM - USS_LINK_RESULT_OFF]);
    out->dtof_ps   = (int32_t)le32(&blk[USS_REG_DTOF_PS   - USS_LINK_RESULT_OFF]);
    out->temp_cC   = (int16_t)le16(&blk[USS_REG_TEMP_CC   - USS_LINK_RESULT_OFF]);
    out->amp_ups   = le16(&blk[USS_REG_AMP_UPS - USS_LINK_RESULT_OFF]);
    out->amp_dns   = le16(&blk[USS_REG_AMP_DNS - USS_LINK_RESULT_OFF]);
    out->snr_db2   = blk[USS_REG_SNR_DB2 - USS_LINK_RESULT_OFF];
    out->gain      = blk[USS_REG_GAIN    - USS_LINK_RESULT_OFF];
    out->vol_ml    = le32(&blk[USS_REG_VOL_ML - USS_LINK_RESULT_OFF]);

    if (out->status & USS_ST_BOOT)
        ESP_LOGW(TAG, "slave rebooted since last contact");
    return true;
}
