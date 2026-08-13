#include "wf280a.h"
#include "i2c_bus.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wf280a";

#define WF_ADDR_ALT     0x38    /* our unit (NVM-reprogrammed) -- try first */
#define WF_ADDR_DEFAULT 0x78
#define WF_ST_BUSY      0x20
#define WF_ST_CRCERR    0x04

#define WF_CMD_PRESS    0xA0
#define WF_CMD_TEMP     0xA4

#define XFER_TO_MS      100
/* Worst-case conversion ~115 ms at OSR x64; poll with headroom. */
#define WF_POLL_MS      5
#define WF_IDLE_TO_MS   400

static i2c_master_dev_handle_t s_dev;

static bool resolve(void)
{
    if (s_dev) return true;
    i2c_master_bus_handle_t bus = eco_i2c_bus();
    if (!bus) return false;
    uint8_t addr = 0;
    if      (i2c_master_probe(bus, WF_ADDR_ALT,     XFER_TO_MS) == ESP_OK) addr = WF_ADDR_ALT;
    else if (i2c_master_probe(bus, WF_ADDR_DEFAULT, XFER_TO_MS) == ESP_OK) addr = WF_ADDR_DEFAULT;
    else return false;
    s_dev = eco_i2c_add(addr);
    if (s_dev) ESP_LOGI(TAG, "found at 0x%02x", addr);
    return s_dev != NULL;
}

/* One triggered conversion: cmd {op,0,0} -> poll busy -> status + data[23:0].
 * Same sequence as the MSP430 firmware's wf_measure() (datasheet sec. 4). */
static bool measure(uint8_t op, uint32_t *raw, uint8_t *st)
{
    uint8_t cmd[3] = { op, 0x00, 0x00 };   /* AFE config from NVM 0x14 */
    if (i2c_master_transmit(s_dev, cmd, 3, XFER_TO_MS) != ESP_OK) return false;

    int waited = 0;
    for (;;) {
        uint8_t s;
        if (i2c_master_receive(s_dev, &s, 1, XFER_TO_MS) != ESP_OK) return false;
        *st = s;
        if (!(s & WF_ST_BUSY)) break;
        if (waited >= WF_IDLE_TO_MS) return false;
        vTaskDelay(pdMS_TO_TICKS(WF_POLL_MS));
        waited += WF_POLL_MS;
    }

    uint8_t b[4];
    if (i2c_master_receive(s_dev, b, 4, XFER_TO_MS) != ESP_OK) return false;
    *st  = b[0];
    *raw = (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    return true;
}

bool wf280a_sample(uint32_t *press_raw, uint32_t *temp_raw, uint8_t *status)
{
    *press_raw = *temp_raw = 0;
    *status = 0;
    if (!resolve()) return false;

    /* Temperature first (datasheet flow), then pressure. */
    if (!measure(WF_CMD_TEMP,  temp_raw,  status)) return false;
    if (!measure(WF_CMD_PRESS, press_raw, status)) return false;

    if (*status & WF_ST_CRCERR)
        ESP_LOGW(TAG, "NVM CRC failed (status 0x%02x)", *status);
    return true;
}
