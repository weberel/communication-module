#include "uss.h"
#include "uss_link.h"
#include "config.h"
#include "i2c_bus.h"

#include <string.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uss";

/* Generous per-transaction timeout: the eUSCI slave clock-stretches while its
 * ISR runs, and may mask its I2C IRQ around the timing-critical ultrasonic
 * capture -- a stalled poll is normal, not an error. */
#define XFER_TO_MS 1000

static i2c_master_dev_handle_t s_dev;

/* HOLD THE CPU OUT OF LIGHT SLEEP FOR EACH TRANSACTION.
 *
 * CONFIG_PM_ENABLE and tickless idle went in on 2026-09-05. sdkconfig.defaults
 * names the hazard for the modem -- "its UART baud derives from a clock that
 * DFS moves, so uplink.c holds a NO_LIGHT_SLEEP + APB_FREQ_MAX lock for the
 * whole session" -- and I2C SCL timing comes off the same clock but was given
 * no lock at all. The USS began dropping off the bus somewhere after 09-02,
 * having not done so once in the preceding eighteen days. */
static esp_pm_lock_handle_t s_pm;
/* Track ownership rather than assuming acquire/release pair up. If the lock is
 * created lazily and creation happens to fail on the acquire but succeed on the
 * release, a naive version releases a lock it never took -- which ESP-IDF
 * reports as INVALID_STATE and, on some builds, aborts. */
static bool s_pm_held;

static void pm_hold(bool on)
{
    if (!s_pm &&
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "uss_i2c", &s_pm) != ESP_OK)
        return;
    if (on && !s_pm_held)      { esp_pm_lock_acquire(s_pm); s_pm_held = true;  }
    else if (!on && s_pm_held) { esp_pm_lock_release(s_pm); s_pm_held = false; }
}

/* RETRY. Before 2026-09-11 these were single-shot, so one failed transfer became
 * "USS silent" and the only response was an 800 ms rail power-cycle -- which
 * reboots the module and zeroes the volume totalizer. One glitch on a 100 kHz
 * bus cost a reboot and the day's volume. */
static bool rd(uint8_t reg, uint8_t *buf, size_t n)
{
    for (int a = 0; a < USS_XFER_RETRIES; a++) {
        esp_err_t e;
        pm_hold(true);
        e = i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, XFER_TO_MS);
        pm_hold(false);
        if (e == ESP_OK) return true;
        if (a + 1 < USS_XFER_RETRIES) vTaskDelay(pdMS_TO_TICKS(USS_XFER_RETRY_MS));
    }
    return false;
}

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    for (int a = 0; a < USS_XFER_RETRIES; a++) {
        esp_err_t e;
        pm_hold(true);
        e = i2c_master_transmit(s_dev, b, 2, XFER_TO_MS);
        pm_hold(false);
        if (e == ESP_OK) return true;
        if (a + 1 < USS_XFER_RETRIES) vTaskDelay(pdMS_TO_TICKS(USS_XFER_RETRY_MS));
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

/* Put the module into autonomous mode: it then measures on its own schedule and
 * integrates flow into USS_REG_VOL_ML, so our sample rate stops dictating its
 * measurement rate. Idempotent -- safe to call every wake, which is what makes
 * it self-healing after the slave reboots (STATUS.AUTO clears on its reset). */
bool uss_start_auto(uint16_t period_s)
{
    uint8_t lo = (uint8_t)(period_s & 0xFF), hi = (uint8_t)(period_s >> 8);
    uint16_t settle_x10 = USS_XT_SETTLE_X10US;
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

    /* USSXT settle, written BEFORE the command because AUTO_START is what
     * applies it (it sets the prime tier, runs the discard, then drops to the
     * operational tier). Writing 0 leaves the module's compiled default, so a
     * module that predates the register is unaffected -- it simply ignores a
     * write to an address it does not decode.
     *
     * This is the lever for the code-135 investigation: the settle is the
     * prime suspect for the regression that began 2026-09-05, and before this
     * register the only way to change it was a reflash, which is impossible on
     * a deployed node. Sweep it here, read the rate back from
     * USS_REG_CAP_BADCODE. */
    if (settle_x10) {
        if (!wr(USS_REG_XT_SETTLE,     (uint8_t)(settle_x10 & 0xFF))) return false;
        if (!wr(USS_REG_XT_SETTLE + 1, (uint8_t)(settle_x10 >> 8)))   return false;
    }

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

/* Stop autonomous mode. On the module this also drops the 5 V boost and both
 * AFE rails, which is the whole point: it is the only load step on that board
 * we can make without cutting the SENSOR rail -- and cutting that rail requires
 * holding the I2C bus low, which deletes the bus the ammeter lives on. */
bool uss_stop_auto(void)
{
    if (!s_dev) {
        eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7);
        if (!s_dev) return false;
    }
    if (!wr(USS_REG_CMD, USS_CMD_AUTO_STOP)) return false;

    /* The command is only queued by the write; the module services it from its
     * main loop. Wait for STATUS.AUTO to clear rather than assuming. */
    for (int waited = 0; waited < USS_AUTO_START_TIMEOUT_MS; waited += USS_POLL_MS) {
        uint8_t st = 0;
        vTaskDelay(pdMS_TO_TICKS(USS_POLL_MS));
        if (!rd(USS_REG_STATUS, &st, 1)) return false;
        if (!(st & USS_ST_AUTO)) return true;
    }
    ESP_LOGW(TAG, "autonomous stop not confirmed");
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
        /* Wait for READY -- but a timeout must NOT mean "power-cycle the rail".
         *
         * The module measures at 1 Hz, so READY is legitimately clear for part
         * of every second and a read can land mid-measurement. Checked against
         * the 2026-09-12 export: the 11 records where the master polled 90-128
         * times ALL SUCCEEDED -- the wait rescued them. Removing it would have
         * turned those into lost samples, and at ~23 % of wakes (11 near-misses
         * + 15 failures out of 113) that is a lot of data to throw away.
         *
         * So the wait stays and gets MORE room, not less. What changes is the
         * consequence: main.c no longer cuts the rail on the first failure, so
         * a slow measurement costs one skipped sample instead of a reboot, a
         * forced abs-ToF re-lock and a zeroed totalizer. */
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
    out->recoveries       = blk[USS_REG_RECOVERIES - USS_LINK_RESULT_OFF];
    out->xt_applied_x10us = le16(&blk[USS_REG_XT_APPLIED - USS_LINK_RESULT_OFF]);
    out->cap_n       = le16(&blk[USS_REG_CAP_N       - USS_LINK_RESULT_OFF]);
    out->cap_badcode = le16(&blk[USS_REG_CAP_BADCODE - USS_LINK_RESULT_OFF]);
    out->cap_badsnr  = le16(&blk[USS_REG_CAP_BADSNR  - USS_LINK_RESULT_OFF]);

    if (out->status & USS_ST_BOOT)
        ESP_LOGW(TAG, "slave rebooted since last contact");
    return true;
}

/* ---- link diagnostics (2026-09-11) --------------------------------------- */

bool uss_read_health(uss_health_t *out)
{
    uint8_t blk[USS_LINK_HEALTH_LEN];

    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!s_dev && !(s_dev = eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7))) return false;
    if (!rd(USS_LINK_HEALTH_OFF, blk, USS_LINK_HEALTH_LEN)) return false;
    /* A module built before this block answers all-zero, which fails the CRC.
     * That is deliberate: "not implemented" must not read as "healthy, zero
     * errors" -- the failure mode that made the capture counters worth adding. */
    if (uss_link_crc8(blk, USS_LINK_HEALTH_LEN - 1) != blk[USS_LINK_HEALTH_LEN - 1])
        return false;

    out->rst_cause = le16(&blk[USS_REG_LH_RSTCAUSE - USS_LINK_HEALTH_OFF]);
    out->starts    = le16(&blk[USS_REG_LH_STARTS   - USS_LINK_HEALTH_OFF]);
    out->stops     = le16(&blk[USS_REG_LH_STOPS    - USS_LINK_HEALTH_OFF]);
    out->rx_bytes  = le16(&blk[USS_REG_LH_RXBYTES  - USS_LINK_HEALTH_OFF]);
    out->tx_bytes  = le16(&blk[USS_REG_LH_TXBYTES  - USS_LINK_HEALTH_OFF]);
    out->cmds      = le16(&blk[USS_REG_LH_CMDS     - USS_LINK_HEALTH_OFF]);
    out->uptime_s  = le16(&blk[USS_REG_LH_UPTIME_S - USS_LINK_HEALTH_OFF]);
    out->last_cmd  = blk[USS_REG_LH_LASTCMD - USS_LINK_HEALTH_OFF];
    return true;
}

/* Probe transfers use a SHORT timeout, unlike the 1 s the normal driver allows.
 *
 * That 1 s is right for a measurement read, where the module may legitimately
 * be busy. It is catastrophic for a probe: 100 transactions x 2 reads x 1 s is
 * 200 s per arm against a 120 s task watchdog with PANIC=y, so a module that
 * has gone quiet turns this diagnostic into a reboot loop -- which is exactly
 * what it did on the first attempt, 2026-09-11, and cost the board's console
 * for twenty minutes. At 100 kHz a 45-byte transfer is ~4 ms, so 50 ms is ~12x
 * headroom and still fails fast. */
#define PROBE_TO_MS 50

static bool rd_probe(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, PROBE_TO_MS) == ESP_OK;
}

void uss_link_probe(uint32_t n, uss_linkstat_t *out)
{
    uss_health_t h0, h1;
    bool have0, have1;
    uint16_t last_seq = 0;
    bool have_seq = false;

    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!s_dev && !(s_dev = eco_i2c_add_tracked(&s_dev, USS_LINK_ADDR7))) return;

    /* The module refreshes its health block about once a second, so bracket the
     * burst with a settle either side; otherwise the delta is taken against a
     * snapshot that predates or straddles the run. */
    have0 = uss_read_health(&h0);
    vTaskDelay(pdMS_TO_TICKS(1200));
    have0 = uss_read_health(&h0) && have0;

    for (uint32_t i = 0; i < n; i++) {
        uint8_t id = 0;
        uint8_t blk[USS_LINK_RESULT_LEN];

        out->attempts++;

        /* Feed the watchdog INSIDE the loop. Between arms is not enough: even
         * at the short probe timeout a fully-failing arm runs for seconds, and
         * the panic-reboot this caused is the reason the timeout above exists. */
        if ((i & 0x0F) == 0) esp_task_wdt_reset();

        /* Cheapest possible probe first: one register, one byte. It separates
         * "could not address the module at all" from "addressed it, the data
         * phase went wrong" -- a distinction uss_sample() throws away. */
        if (!rd_probe(USS_REG_WHO_AM_I, &id, 1)) { out->err_tx++; continue; }
        if (id != USS_LINK_WHOAMI)               { out->err_id++; continue; }

        if (!rd_probe(USS_LINK_RESULT_OFF, blk, USS_LINK_RESULT_LEN)) { out->err_rx++; continue; }
        if (uss_link_crc8(blk, USS_LINK_RESULT_LEN - 1) != blk[USS_LINK_RESULT_LEN - 1]) {
            out->err_crc++;
            continue;
        }

        /* CRC-valid but standing still: the module is answering the bus while
         * no longer measuring. Invisible today -- seq is read and never
         * compared -- and it would look identical to perfect health. */
        {
            uint16_t seq = le16(&blk[USS_REG_SEQ - USS_LINK_RESULT_OFF]);
            if (have_seq && seq == last_seq) out->err_stale++;
            last_seq = seq;
            have_seq = true;
        }
        out->ok++;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    have1 = uss_read_health(&h1);
    if (have0 && have1)
        out->slave_starts_delta = (uint16_t)(h1.starts - h0.starts);
}
