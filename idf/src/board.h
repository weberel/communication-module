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
#define ECO_PIN_SENSOR_PWR   14   /* external sensor rail, active HIGH */
#define ECO_PIN_MODEM_PWRKEY 22   /* via inverter: HIGH = asserted */
#define ECO_PIN_MODEM_PWR_EN 23   /* HIGH = modem rail on */

/* Safe GPIO state: modem rail off, PWRKEY idle-high, sensor rail off, LED off.
 * Releases sleep holds first. Call once at the top of app_main. */
void board_init(void);

/* Deep sleep with rails held safe. PWRKEY is held LOW (deasserted) through
 * sleep -- holding it asserted burns ~0.3 mA in the Q409 inverter (dl-2.10
 * lesson). Wakes on timer or the QON button (EXT1). Never returns. */
void board_deep_sleep(uint32_t seconds);

bool board_woke_from_timer(void);
bool board_woke_from_button(void);

#ifdef __cplusplus
}
#endif
