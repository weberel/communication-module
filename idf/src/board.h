/*
 * board.h  --  pin init + deep-sleep helper, ESP-IDF port of EcoTraceBoard.
 * Pin map source of truth: lib/EcoTrace/ecotrace_pins.h (Rev A).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECO_PIN_LED          1
#define ECO_PIN_QON          2    /* button / BQ QON, external pull-up, EXT1 wake */
#define ECO_PIN_INT_SHARED   3    /* shared sensor INT (SC7A20 + LTR-303), external
                                   * pull-up, ACTIVE LOW, EXT1 wake. GPIO0-7 are the
                                   * LP-capable pins on the C6, so 3 can wake deep sleep. */
#define ECO_PIN_SENSOR_PWR   14   /* external sensor rail, active HIGH */
#define ECO_PIN_MODEM_PWRKEY 22   /* via inverter: HIGH = asserted */
#define ECO_PIN_MODEM_PWR_EN 23   /* HIGH = modem rail on */

/* Safe GPIO state: modem rail off, PWRKEY idle-high, sensor rail off, LED off.
 * Releases sleep holds first. Call once at the top of app_main. */
void board_init(void);

/* Milliseconds to allow the ultrasonic slave to boot after its rail returns.
 *
 * 3000, not 600. That board's startup is genuinely slow: hal init, the USS
 * configuration, and then USSXT crystal priming which retries up to 5 times at
 * 300 ms each before the HSPLL is usable. Measured 2026-08-15: talking to it
 * ~470 ms after the rail came up gets a clean NACK because its I2C slave has
 * not been initialised yet -- which looks exactly like a dead module. */
#define BOARD_SENSOR_BOOT_MS  3000

/* Switched SENSOR rail (GPIO14, active HIGH -- the J404 header only; the
 * on-board sensors live on the always-on LDO). board_init() leaves it OFF, so
 * anything on that header must be switched on explicitly before it is used. */
void board_sensor_power(bool on);

/* Power-cycle the switched SENSOR rail (GPIO14) and everything on it.
 * Drives SDA/SCL low for the whole off window, because otherwise the bus
 * back-feeds the slave through its ESD clamps and the rail never falls.
 * The I2C bus is torn down and rebuilt around this, so it must not overlap
 * any other sensor access. ~off_ms + 600 ms total.
 *
 * off_ms must be generous: the rail is a high-side switch with NO bleed
 * resistor, so the node floats when it opens and discharges only through the
 * load. Measured with an LED across the header, it fades rather than switching
 * off. A respin should add ~100k to GND on that node. */
void board_sensor_power_cycle(uint32_t off_ms);

/* Arm (or disarm) the shared sensor INT as a deep-sleep wake source. Off by
 * default: a board with nothing driving that line must not be woken by it. */
void board_set_motion_wake(bool enable);

/* Was this wake caused by the shared sensor INT rather than the button/timer? */
bool board_woke_from_motion(void);

/* Deep sleep with rails held safe. PWRKEY is held LOW (deasserted) through
 * sleep -- holding it asserted burns ~0.3 mA in the Q409 inverter (dl-2.10
 * lesson). Wakes on timer or the QON button (EXT1). Never returns. */
void board_deep_sleep(uint32_t seconds);

bool board_woke_from_timer(void);
bool board_woke_from_button(void);

#ifdef __cplusplus
}
#endif
