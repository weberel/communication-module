/*
 * wf280a.h  --  WF280A digital pressure sensor (U8 on the ultrasonic PCB,
 * shared I2C bus). Command-triggered conversions; the compensation polynomial
 * is vendor-private, so this driver reports RAW 24-bit counts (same decision
 * as the MSP430 firmware's PRS command) -- conversion happens server-side
 * once the vendor coefficients are characterized.
 *
 * Address: datasheet default 0x78, but NVM word 0x02 reprograms it and our
 * unit answers at 0x38 -- the driver probes 0x38 first and caches the winner.
 * Optional like every other sensor: absent = two NACKs, degrade to zeros.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One temperature + one pressure conversion, raw counts (data[23:0]).
 * ~30-250 ms depending on the sensor's NVM-configured OSR. status is the
 * last status byte (bit2 = NVM CRC failed -- log-worthy, not fatal). */
bool wf280a_sample(uint32_t *press_raw, uint32_t *temp_raw, uint8_t *status);

#ifdef __cplusplus
}
#endif
