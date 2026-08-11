#include "bq25792.h"
#include "i2c_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Register map (subset in use; see Arduino header for the full annotated map). */
#define REG_VREG      0x01
#define REG_ICHG      0x03
#define REG_VINDPM    0x05
#define REG_IINDPM    0x06
#define REG_CHG_CTRL0 0x0F   /* EN_CHG[5] EN_HIZ[2] */
#define REG_CHG_CTRL1 0x10   /* WATCHDOG[2:0] */
#define REG_CHG_CTRL2 0x11   /* SDRV_CTRL[2:1] SDRV_DLY[0] */
#define REG_CHG_CTRL4 0x13   /* EN_ACDRV2[7] EN_ACDRV1[6] */
#define REG_CHG_CTRL5 0x14   /* SFET_PRESENT[7] EN_IBAT[5] EN_EXTILIM[1] */
#define REG_STATUS0   0x1B
#define REG_STATUS1   0x1C
#define REG_FAULT0    0x20
#define REG_ADC_CTRL  0x2E
#define REG_IBUS_ADC  0x31
#define REG_IBAT_ADC  0x33
#define REG_VBUS_ADC  0x35
#define REG_VAC1_ADC  0x37
#define REG_VAC2_ADC  0x39
#define REG_VBAT_ADC  0x3B
#define REG_VSYS_ADC  0x3D
#define REG_PART_INFO 0x48

#define XFER_MS 100

static i2c_master_dev_handle_t s_dev;

static bool rd(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!s_dev) return false;
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, XFER_MS) == ESP_OK;
}

static uint8_t rd8(uint8_t reg)
{
    uint8_t v = 0;
    rd(reg, &v, 1);
    return v;
}

static uint16_t rd16(uint8_t reg)   /* big-endian */
{
    uint8_t b[2] = { 0, 0 };
    rd(reg, b, 2);
    return ((uint16_t)b[0] << 8) | b[1];
}

static bool wr8(uint8_t reg, uint8_t v)
{
    if (!s_dev) return false;
    uint8_t tx[2] = { reg, v };
    return i2c_master_transmit(s_dev, tx, 2, XFER_MS) == ESP_OK;
}

static bool wr16(uint8_t reg, uint16_t v)   /* MSB first */
{
    if (!s_dev) return false;
    uint8_t tx[3] = { reg, (uint8_t)(v >> 8), (uint8_t)v };
    return i2c_master_transmit(s_dev, tx, 3, XFER_MS) == ESP_OK;
}

static bool set_bits(uint8_t reg, uint8_t set, uint8_t clr)
{
    uint8_t v = 0;
    if (!rd(reg, &v, 1)) return false;
    v = (v | set) & ~clr;
    return wr8(reg, v);
}

bool bq_begin(void)
{
    if (!s_dev) s_dev = eco_i2c_add(BQ25792_ADDR);
    if (!s_dev) return false;
    uint8_t v = 0;
    if (!rd(REG_PART_INFO, &v, 1)) return false;
    return ((v >> 3) & 0x07) == 0x01;   /* PN = BQ25792 */
}

void bq_adc_enable(bool continuous)
{
    wr8(REG_ADC_CTRL, continuous ? 0x80 : 0xC0);   /* 15-bit resolution */
    vTaskDelay(pdMS_TO_TICKS(150));   /* first continuous conversion settle */
}

void bq_adc_disable(void) { wr8(REG_ADC_CTRL, 0x00); }

void bq_ibat_sense(bool on)
{
    set_bits(REG_CHG_CTRL5, on ? 0x20 : 0x00, on ? 0x00 : 0x20);
}

uint16_t bq_vbat_mv(void) { return rd16(REG_VBAT_ADC); }
uint16_t bq_vbus_mv(void) { return rd16(REG_VBUS_ADC); }
uint16_t bq_vsys_mv(void) { return rd16(REG_VSYS_ADC); }
uint16_t bq_vac1_mv(void) { return rd16(REG_VAC1_ADC); }
uint16_t bq_vac2_mv(void) { return rd16(REG_VAC2_ADC); }
int16_t  bq_ibat_ma(void) { return (int16_t)rd16(REG_IBAT_ADC); }
int16_t  bq_ibus_ma(void) { return (int16_t)rd16(REG_IBUS_ADC); }

bq_chg_stat_t bq_charge_state(void) { return (bq_chg_stat_t)((rd8(REG_STATUS1) >> 5) & 0x07); }
bool bq_vbus_present(void) { return (rd8(REG_STATUS0) & 0x01) != 0; }
bool bq_ac1_present(void)  { return (rd8(REG_STATUS0) & 0x02) != 0; }
bool bq_ac2_present(void)  { return (rd8(REG_STATUS0) & 0x04) != 0; }

void bq_faults(uint8_t *fault0, uint8_t *fault1)
{
    uint8_t f[2] = { 0, 0 };
    rd(REG_FAULT0, f, 2);
    *fault0 = f[0];
    *fault1 = f[1];
}

void bq_set_vreg_mv(uint16_t mv)   { wr16(REG_VREG,   (mv / 10) & 0x07FF); }
void bq_set_ichg_ma(uint16_t ma)   { wr16(REG_ICHG,   (ma / 10) & 0x01FF); }
void bq_set_iindpm_ma(uint16_t ma) { wr16(REG_IINDPM, (ma / 10) & 0x01FF); }

void bq_set_vindpm_mv(uint16_t mv)
{
    if (mv < 3600) mv = 3600;   /* clamped low per datasheet */
    wr8(REG_VINDPM, (uint8_t)(mv / 100));
}

void bq_set_hiz(bool on)         { set_bits(REG_CHG_CTRL0, on ? 0x04 : 0x00, on ? 0x00 : 0x04); }
void bq_enable_charging(bool on) { set_bits(REG_CHG_CTRL0, on ? 0x20 : 0x00, on ? 0x00 : 0x20); }
void bq_disable_watchdog(void)   { set_bits(REG_CHG_CTRL1, 0x00, 0x07); }
void bq_enable_acdrv1(bool on)   { set_bits(REG_CHG_CTRL4, on ? 0x40 : 0x00, on ? 0x00 : 0x40); }
void bq_enable_acdrv2(bool on)   { set_bits(REG_CHG_CTRL4, on ? 0x80 : 0x00, on ? 0x00 : 0x80); }

static void bq_ext_ilim(bool on) { set_bits(REG_CHG_CTRL5, on ? 0x02 : 0x00, on ? 0x00 : 0x02); }
static void bq_sfet_present(void){ set_bits(REG_CHG_CTRL5, 0x80, 0x00); }

void bq_configure_charging(uint16_t ichg_ma, uint16_t iindpm_ma, uint16_t vreg_mv)
{
    bq_disable_watchdog();   /* keep our settings past the 40 s charger WD */
    bq_set_hiz(false);
    bq_ext_ilim(false);      /* IINDPM register is the sole input limit (issue #3) */
    if (vreg_mv)   bq_set_vreg_mv(vreg_mv);
    if (ichg_ma)   bq_set_ichg_ma(ichg_ma);
    if (iindpm_ma) bq_set_iindpm_ma(iindpm_ma);
    bq_enable_charging(true);
}

void bq_enter_ship_mode(void)
{
    /* SFET_PRESENT MUST be set first -- without it the SDRV_CTRL write is
     * ACKed and silently discarded (cost a full debug day on 2026-08-07). */
    bq_sfet_present();
    bq_disable_watchdog();
    uint8_t v = rd8(REG_CHG_CTRL2);
    v = (v & ~0x07) | (0x02 << 1) | 0x01;   /* ship, no 10 s delay */
    wr8(REG_CHG_CTRL2, v);
}

void bq_system_power_reset(void)
{
    bq_sfet_present();
    bq_disable_watchdog();
    uint8_t v = rd8(REG_CHG_CTRL2);
    v = (v & ~0x07) | (0x03 << 1) | 0x01;
    wr8(REG_CHG_CTRL2, v);
}
