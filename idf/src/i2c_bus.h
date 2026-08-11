/*
 * i2c_bus.h  --  the shared I2C bus (GPIO6/7, 100 kHz), ESP-IDF i2c_master port.
 * One bus, lazily initialised; every peripheral driver registers its own device
 * handle on it. Pin map source of truth: lib/EcoTrace/ecotrace_pins.h (Rev A).
 */
#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ECO_I2C_SDA     6
#define ECO_I2C_SCL     7
#define ECO_I2C_HZ      100000

/* Returns the bus handle, initialising it on first call. NULL on failure. */
i2c_master_bus_handle_t eco_i2c_bus(void);

/* Convenience: add a device at addr, standard speed. NULL on failure. */
i2c_master_dev_handle_t eco_i2c_add(uint8_t addr7);

#ifdef __cplusplus
}
#endif
