/*
 * sensors.h  --  the on-board I2C sensors, ESP-IDF port.
 * LTR-303 light (0x29), SC7A20 accel (0x18/0x19), MS5837-02BA barometer (0x76).
 * Register sequences identical to the hardware-validated Arduino drivers.
 * All are optional: absent sensors degrade to zeros, never errors.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LTR-303: activate, read both channels, back to standby. ~150 ms blocking. */
bool ltr303_sample(uint16_t *ch0, uint16_t *ch1);

/* SC7A20: power up, one X/Y/Z reading in milli-g, power down. ~30 ms. */
bool sc7a20_sample(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

/* MS5837: reset + PROM (CRC-4 checked) + one P/T conversion. ~60 ms. */
bool ms5837_sample(float *mbar, float *degc);

#ifdef __cplusplus
}
#endif
