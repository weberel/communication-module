/*
 * bq25792.h  --  TI BQ25792 buck-boost charger / PMIC, ESP-IDF port.
 * Register map and scaling identical to the hardware-validated Arduino driver
 * (lib/EcoTrace/BQ25792.*, TI datasheet SLUSDG1D). All multi-byte registers
 * big-endian; IBAT/IBUS are 16-bit two's-complement.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BQ25792_ADDR 0x6B

typedef enum {
    BQ_CHG_NOT_CHARGING = 0, BQ_CHG_TRICKLE = 1, BQ_CHG_PRECHARGE = 2,
    BQ_CHG_FAST_CC = 3, BQ_CHG_TAPER_CV = 4, BQ_CHG_RESERVED = 5,
    BQ_CHG_TOPOFF = 6, BQ_CHG_DONE = 7,
} bq_chg_stat_t;

bool bq_begin(void);                 /* true if a BQ25792 (PN=001) answers */

/* ---- ADC ---- */
void bq_adc_enable(bool continuous); /* blocks ~150 ms for first conversion */
void bq_adc_disable(void);
void bq_ibat_sense(bool on);         /* EN_IBAT: off across sleep (quiescent) */

uint16_t bq_vbat_mv(void);
uint16_t bq_vbus_mv(void);
uint16_t bq_vsys_mv(void);
uint16_t bq_vac1_mv(void);
uint16_t bq_vac2_mv(void);
int16_t  bq_ibat_ma(void);           /* + charge, - discharge */
int16_t  bq_ibus_ma(void);

/* ---- status / faults ---- */
bq_chg_stat_t bq_charge_state(void);
bool bq_vbus_present(void);
bool bq_ac1_present(void);           /* USB path */
bool bq_ac2_present(void);           /* solar path */
void bq_faults(uint8_t *fault0, uint8_t *fault1);

/* ---- thermal diagnostics (the 2026-08-12 JEITA-lockout lesson) ---- */
int16_t  bq_tdie_dC(void);           /* charger die temp, 0.1 C units */
uint16_t bq_ts_pct_x100(void);       /* TS pin (battery NTC) as % of bias x100 */
uint8_t  bq_ts_stat(void);           /* Charger_Status_4: TS_COLD/COOL/WARM/HOT bits */

/* ---- limits / control ---- */
void bq_set_vreg_mv(uint16_t mv);
void bq_set_ichg_ma(uint16_t ma);
void bq_set_iindpm_ma(uint16_t ma);
void bq_set_vindpm_mv(uint16_t mv);  /* clamped >= 3600 per datasheet */
void bq_set_hiz(bool on);
void bq_enable_charging(bool on);
void bq_disable_watchdog(void);
void bq_enable_acdrv1(bool on);
void bq_enable_acdrv2(bool on);

/* disableWatchdog + HIZ off + ext-ILIM off + limits + EN_CHG (0 = keep). */
void bq_configure_charging(uint16_t ichg_ma, uint16_t iindpm_ma, uint16_t vreg_mv);

/* Ship mode / power reset. Sets SFET_PRESENT (REG14[7]) first -- without it
 * every SDRV_CTRL write is silently discarded (hardware-measured 2026-08-07). */
void bq_enter_ship_mode(void);
void bq_system_power_reset(void);

#ifdef __cplusplus
}
#endif
