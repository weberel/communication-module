#include "i2c_bus.h"

static i2c_master_bus_handle_t s_bus;

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
