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

/* Attempts per transaction, and the pause between them.
 *
 * There was NO retry here, and that asymmetry is what made the link brittle:
 * a single glitched byte failed the read, the failure escalated to a rail
 * cycle, the module rebooted, and the reboot reset the volume totalizer and
 * forced a fresh abs-ToF search that could lock the wrong lobe. One bad byte
 * cost a measurement, the running total, and sometimes a code-135 latch.
 *
 * Three attempts because the slave legitimately clock-stretches around a
 * capture; the gap lets a busy window pass rather than hammering it. */
#define XFER_TRIES   3
#define XFER_GAP_MS  5

static bool rd(uint8_t reg, uint8_t *buf, size_t n)
{
    for (int i = 0; i < XFER_TRIES; i++) {
        if (i) vTaskDelay(pdMS_TO_TICKS(XFER_GAP_MS));
        if (i2c_master_transmit_receive(s_dev, &reg, 1, buf, n,
                                        XFER_TO_MS) == ESP_OK) {
            if (i) ESP_LOGW(TAG, "read 0x%02X ok on attempt %d", reg, i + 1);
            return true;
        }
    }
    return false;
}

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    for (int i = 0; i < XFER_TRIES; i++) {
        if (i) vTaskDelay(pdMS_TO_TICKS(XFER_GAP_MS));
        if (i2c_master_transmit(s_dev, b, 2, XFER_TO_MS) == ESP_OK) {
            if (i) ESP_LOGW(TAG, "write 0x%02X ok on attempt %d", reg, i + 1);
            return true;
        }
    }
    return false;
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

/* Read the link-health block (0x30..0x3F, its own CRC).
 *
 * Separate from the result block on purpose: this must be readable when a
 * measurement read has just failed, which is exactly when it is worth having.
 * The module's UART cannot answer -- it is on the comm board, and LPM3 stops
 * SMCLK so the UART cannot receive -- so I2C is the only channel that can say
 * why it restarted.
 *
 * Reading it is also the one way to tell two very different faults apart:
 *   delta(starts) ~= transactions attempted -> the bus reached the module and
 *       the fault is above the physical layer (CRC, timing, slave state);
 *   delta(starts) ~= 0                      -> we never got on the bus at all
 *       (wiring, pull-ups, a wedged master, the rail down). */
bool uss_read_health(uss_health_t *out)
{
    uint8_t blk[USS_LH_LEN];
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    if (!rd(USS_LH_OFF, blk, sizeof(blk))) return false;
    if (uss_link_crc8(blk, USS_LH_LEN - 1) != blk[USS_LH_LEN - 1]) {
        ESP_LOGW(TAG, "health CRC mismatch");
        return false;
    }
    out->rst_cause = le16(&blk[USS_REG_LH_RSTCAUSE - USS_LH_OFF]);
    out->starts    = le16(&blk[USS_REG_LH_STARTS   - USS_LH_OFF]);
    out->stops     = le16(&blk[USS_REG_LH_STOPS    - USS_LH_OFF]);
    out->rx_bytes  = le16(&blk[USS_REG_LH_RXBYTES  - USS_LH_OFF]);
    out->tx_bytes  = le16(&blk[USS_REG_LH_TXBYTES  - USS_LH_OFF]);
    out->cmds      = le16(&blk[USS_REG_LH_CMDS     - USS_LH_OFF]);
    out->uptime_s  = le16(&blk[USS_REG_LH_UPTIME_S - USS_LH_OFF]);
    out->last_cmd  = blk[USS_REG_LH_LASTCMD - USS_LH_OFF];
    return true;
}

/* Reset the slave state machine WITHOUT dropping its rail.
 *
 * The rung above a rail cycle in the recovery ladder. It clears a wedged slave
 * while leaving the module powered, so the abs-ToF lock is re-searched but the
 * persisted totalizer survives -- which is the whole reason to prefer it. */
bool uss_soft_reset(void)
{
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    return wr(USS_REG_CMD, USS_CMD_SOFT_RESET);
}

/* Zero the module's volume totalizer. Needed now that the total PERSISTS in the
 * module's FRAM and AUTO_START no longer clears it -- which is the point: a
 * recovery power-cycle used to throw the running total away every time. */
bool uss_reset_volume(void)
{
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    return wr(USS_REG_CMD, USS_CMD_VOL_RESET);
}

/* Null the module's dTOF offset AT THE CURRENT FLOW, and persist it there.
 *
 * ONLY call this when flow is genuinely zero: the module cannot verify that, so
 * a zerocal sent while gas moves calibrates the flow away permanently. Exposed
 * because a deployed module otherwise has no calibration path at all. */
bool uss_zerocal(void)
{
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    return wr(USS_REG_CMD, USS_CMD_ZEROCAL);
}

/* Put the module into autonomous mode: it then measures on its own schedule and
 * integrates flow into USS_REG_VOL_ML, so our sample rate stops dictating its
 * measurement rate. Idempotent -- safe to call every wake, which is what makes
 * it self-healing after the slave reboots (STATUS.AUTO clears on its reset). */
bool uss_start_auto(uint16_t period_s)
{
    uint8_t lo = (uint8_t)(period_s & 0xFF), hi = (uint8_t)(period_s >> 8);
    uint8_t st = 0;
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    /* Only start it if it is NOT already running. USS_CMD_AUTO_START zeroes the
     * totalizer, so re-issuing it every wake would throw away the volume
     * accumulated since the last read -- the one thing this mode exists to
     * provide. Re-asserting only when STATUS.AUTO is clear still recovers
     * automatically after the module reboots. */
    if (!rd(USS_REG_STATUS, &st, 1)) return false;
    if (st & USS_ST_AUTO) return true;

    if (!wr(USS_REG_AUTO_PERIOD, lo))     return false;
    if (!wr(USS_REG_AUTO_PERIOD + 1, hi)) return false;
    if (!wr(USS_REG_CMD, USS_CMD_AUTO_START)) return false;

    /* WAIT for STATUS.AUTO before returning.
     *
     * The command is only queued by the write; the module then brings its AFE
     * rails up, settles them and runs one discard capture before it sets the
     * bit -- about 700 ms. Returning immediately let uss_sample() read STATUS
     * inside that window, see AUTO clear, and issue a MEASURE, which is exactly
     * the interleaving that stalls the module's own cadence. */
    for (int waited = 0; waited < USS_AUTO_START_TIMEOUT_MS; waited += USS_POLL_MS) {
        vTaskDelay(pdMS_TO_TICKS(USS_POLL_MS));
        if (!rd(USS_REG_STATUS, &st, 1)) return false;
        if (st & USS_ST_AUTO) {
            ESP_LOGI(TAG, "autonomous mode started, period %u s", period_s);
            return true;
        }
    }
    ESP_LOGW(TAG, "autonomous start not confirmed (status 0x%02x)", st);
    return false;
}

bool uss_sample(uss_result_t *out)
{
    memset(out, 0, sizeof(*out));

    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }

    /* Identify -- doubles as the absence probe (absent module = one NACK). */
    uint8_t id[2];
    if (!rd(USS_REG_WHO_AM_I, id, 2)) return false;
    if (id[0] != USS_LINK_WHOAMI || id[1] != USS_LINK_PROTO) {
        ESP_LOGW(TAG, "bad identity 0x%02x proto %u", id[0], id[1]);
        return false;
    }

    uint8_t st = 0;
    if (!rd(USS_REG_STATUS, &st, 1)) return false;

    /* If the module is running autonomously, do NOT command a measurement --
     * just take whatever it last latched. Interleaving our own MEASURE with its
     * schedule would stall its cadence and corrupt the integration interval. */
    if (st & USS_ST_AUTO) {
        /* AUTO is set the moment the module accepts the command, but READY only
         * appears when its first autonomous measurement publishes -- up to one
         * period later. Sampling inside that window used to return "module not
         * answering" for a module that was working perfectly. Wait for it. */
        int waited = 0;
        while (!(st & USS_ST_READY)) {
            if (waited >= USS_MEAS_TIMEOUT_MS) {
                ESP_LOGW(TAG, "auto mode but no result within %d ms (status 0x%02x)",
                         USS_MEAS_TIMEOUT_MS, st);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(USS_POLL_MS));
            waited += USS_POLL_MS;
            if (!rd(USS_REG_STATUS, &st, 1)) return false;
        }
    } else {
        if (!wr(USS_REG_CMD, USS_CMD_MEASURE)) return false;

        /* Poll STATUS until READY. First wait is longer: the slave is doing the
         * whole capture + algorithm run, no point hammering the bus. */
        vTaskDelay(pdMS_TO_TICKS(100));
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
    out->tof_ups_q40 = le32(&blk[USS_REG_TOF_UPS_Q40 - USS_LINK_RESULT_OFF]);
    out->tof_dns_q40 = le32(&blk[USS_REG_TOF_DNS_Q40 - USS_LINK_RESULT_OFF]);

    if (out->status & USS_ST_BOOT)
        ESP_LOGW(TAG, "slave rebooted since last contact");
    return true;
}
