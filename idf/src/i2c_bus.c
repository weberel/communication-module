#include "i2c_bus.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_bus_handle_t s_bus;

/* Where each driver keeps its cached device handle.
 *
 * The bus has to be torn down to power-cycle the ultrasonic board (see
 * eco_i2c_hold_low), and every handle registered on it dies with it. Rather
 * than have each driver poll for that, they hand us the address of their
 * cached handle and we null it on teardown -- their existing lazy
 * `if (!h) h = eco_i2c_add(...)` then re-attaches on the next call. */
#define ECO_I2C_MAX_TRACKED 8
static i2c_master_dev_handle_t *s_slots[ECO_I2C_MAX_TRACKED];
static int s_nslots;

i2c_master_bus_handle_t eco_i2c_bus(void)
{
    if (s_bus) return s_bus;

    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,          /* auto */
        .sda_io_num = ECO_I2C_SDA,
        .scl_io_num = ECO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* board has external 4k7 pull-ups to 3V3 */
        .flags.enable_internal_pullup = false,
    };
    if (i2c_new_master_bus(&cfg, &s_bus) != ESP_OK) s_bus = NULL;
    return s_bus;
}

i2c_master_dev_handle_t eco_i2c_add(uint8_t addr7)
{
    i2c_master_bus_handle_t bus = eco_i2c_bus();
    if (!bus) return NULL;

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr7,
        .scl_speed_hz = ECO_I2C_HZ,
    };
    i2c_master_dev_handle_t h = NULL;
    if (i2c_master_bus_add_device(bus, &dev, &h) != ESP_OK) return NULL;
    return h;
}

i2c_master_dev_handle_t eco_i2c_add_tracked(i2c_master_dev_handle_t *slot, uint8_t addr7)
{
    i2c_master_dev_handle_t h = eco_i2c_add(addr7);
    if (!h || !slot) return h;

    for (int i = 0; i < s_nslots; i++)
        if (s_slots[i] == slot) { *slot = h; return h; }

    if (s_nslots < ECO_I2C_MAX_TRACKED) s_slots[s_nslots++] = slot;
    *slot = h;
    return h;
}

/* Tear the bus down and hold SDA+SCL LOW as plain GPIOs.
 *
 * WHY THIS IS NEEDED (measured 2026-08-15): cutting the ultrasonic board's VCC
 * does NOT power it down while the bus idles high. Current flows our 3V3 ->
 * pull-up -> SDA -> that board's ESD clamp -> its VCC, parking it a diode drop
 * below the bus: too low to run, too high to trigger a power-on reset. Without
 * this the sensor rail switch cannot actually reset the slave, and a node that
 * cannot reset its sensor cannot recover remotely.
 *
 * CAUTION: this bus is shared with the charger, the light sensor, the
 * accelerometer and the barometer. Holding it low blocks all of them, so keep
 * the window short and never overlap it with a charger poll. */
void eco_i2c_hold_low(void)
{
    for (int i = 0; i < s_nslots; i++) {
        if (s_slots[i] && *s_slots[i]) {
            i2c_master_bus_rm_device(*s_slots[i]);
            *s_slots[i] = NULL;
        }
    }
    if (s_bus) { i2c_del_master_bus(s_bus); s_bus = NULL; }

    const gpio_num_t pins[2] = { ECO_I2C_SDA, ECO_I2C_SCL };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
}

void eco_i2c_release(void)
{
    const gpio_num_t pins[2] = { ECO_I2C_SDA, ECO_I2C_SCL };
    for (int i = 0; i < 2; i++) {
        gpio_set_level(pins[i], 1);
        gpio_reset_pin(pins[i]);            /* back to high-Z for the driver */
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    (void) eco_i2c_bus();                   /* rebuild; devices re-attach lazily */
}

/* Probe the whole 7-bit range and log what answers.
 *
 * Worth doing every wake: it costs a few tens of ms (absent devices NACK
 * immediately, they do not time out) and it turns "the reading is zero" into
 * "that chip is not on the bus", which are completely different faults. */
void eco_i2c_scan(void)
{
    i2c_master_bus_handle_t bus = eco_i2c_bus();
    if (!bus) { ESP_LOGE("i2cscan", "no bus"); return; }

    char line[160];
    int n = 0, len = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(bus, a, 10) != ESP_OK) continue;
        const char *who =
            (a == 0x18 || a == 0x19) ? "SC7A20"   :
            (a == 0x29)              ? "LTR303"   :
            (a == 0x2C)              ? "USS"      :
            (a == 0x35 || a == 0x60) ? "ATECC608" :
            (a == 0x38 || a == 0x78) ? "WF280A"   :
            (a == 0x6B)              ? "BQ25792"  :
            (a == 0x76 || a == 0x77) ? "MS5837"   : "?";
        if (len < (int)sizeof(line) - 24)
            len += snprintf(line + len, sizeof(line) - len, "0x%02X:%s ", a, who);
        n++;
    }
    if (!n) ESP_LOGW("i2cscan", "NOTHING on the bus");
    else    ESP_LOGI("i2cscan", "%d device(s): %s", n, line);
}
