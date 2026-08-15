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

/* Same, but remembers where the caller caches the handle so a bus teardown can
 * null it. Prefer this: it is what makes eco_i2c_hold_low() safe. */
i2c_master_dev_handle_t eco_i2c_add_tracked(i2c_master_dev_handle_t *slot, uint8_t addr7);

/* Power-cycle support for the ultrasonic board. hold_low() removes every device,
 * deletes the bus and drives SDA+SCL low so the slave's rail can actually fall
 * (its ESD clamps back-feed from an idle-high bus otherwise). release() restores
 * the pins and rebuilds the bus; drivers re-attach on their next call.
 * The bus is shared -- keep the window short. */
/* Probe 0x08..0x77 and log what answers, with names. Cheap; run it each wake
 * so a missing chip names itself instead of showing up as a zero reading. */
void eco_i2c_scan(void);

void eco_i2c_hold_low(void);
void eco_i2c_release(void);

#ifdef __cplusplus
}
#endif
