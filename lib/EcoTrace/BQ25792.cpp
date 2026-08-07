#include "BQ25792.h"
#include <string.h>

/* ===================== raw register access ===================== */
bool BQ25792::readReg(uint8_t reg, uint8_t* buf, size_t len)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    size_t got = _bus->requestFrom((int)_addr, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = _bus->read();
    return true;
}

uint8_t BQ25792::readReg8(uint8_t reg)
{
    uint8_t v = 0;
    readReg(reg, &v, 1);
    return v;
}

uint16_t BQ25792::readReg16(uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    readReg(reg, b, 2);
    return ((uint16_t)b[0] << 8) | b[1];   /* big-endian */
}

bool BQ25792::writeReg8(uint8_t reg, uint8_t value)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(value);
    return _bus->endTransmission(true) == 0;
}

bool BQ25792::writeReg16(uint8_t reg, uint16_t value)
{
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write((uint8_t)(value >> 8));    /* MSB first */
    _bus->write((uint8_t)(value & 0xFF));
    return _bus->endTransmission(true) == 0;
}

bool BQ25792::setBits(uint8_t reg, uint8_t set, uint8_t clr)
{
    uint8_t v = 0;
    if (!readReg(reg, &v, 1)) return false;
    v = (v | set) & ~clr;
    return writeReg8(reg, v);
}

/* ===================== presence ===================== */
bool BQ25792::isPresent()
{
    uint8_t v = 0;
    if (!readReg(REG_PART_INFO, &v, 1)) return false;
    return ((v >> 3) & 0x07) == 0b001;   /* PN = BQ25792 */
}

bool BQ25792::begin()
{
    return isPresent();
}

/* ===================== ADC ===================== */
void BQ25792::enableADC(bool continuous)
{
    /* ADC_EN (bit7), rate = continuous(0)/one-shot(1) bit6, sample bits[5:4]=00
     * (15-bit, most accurate). */
    writeReg8(REG_ADC_CTRL, continuous ? 0x80 : 0xC0);
    delay(150);   /* first continuous conversion needs ~ this long to be valid */
}

void BQ25792::disableADC()
{
    writeReg8(REG_ADC_CTRL, 0x00);
}

void BQ25792::enableIbatSensing(bool on)
{
    setBits(REG_CHG_CTRL5, on ? 0x20 : 0x00, on ? 0x00 : 0x20);
}

uint16_t BQ25792::readVbat_mV() { return readReg16(REG_VBAT_ADC); }
uint16_t BQ25792::readVbus_mV() { return readReg16(REG_VBUS_ADC); }
uint16_t BQ25792::readVsys_mV() { return readReg16(REG_VSYS_ADC); }
uint16_t BQ25792::readVac1_mV() { return readReg16(REG_VAC1_ADC); }
uint16_t BQ25792::readVac2_mV() { return readReg16(REG_VAC2_ADC); }
int16_t  BQ25792::readIbat_mA() { return (int16_t)readReg16(REG_IBAT_ADC); }
int16_t  BQ25792::readIbus_mA() { return (int16_t)readReg16(REG_IBUS_ADC); }

/* ===================== status ===================== */
BQ25792::ChgStat BQ25792::chargeState()
{
    return (ChgStat)((readReg8(REG_STATUS1) >> 5) & 0x07);
}

const char* BQ25792::chargeStateName()
{
    static const char* names[] = {
        "not charging", "trickle", "pre-charge", "fast (CC)",
        "taper (CV)", "reserved", "top-off", "done"
    };
    return names[chargeState()];
}

bool BQ25792::vbusPresent() { return (readReg8(REG_STATUS0) & 0x01) != 0; }
bool BQ25792::ac1Present()  { return (readReg8(REG_STATUS0) & 0x02) != 0; }
bool BQ25792::ac2Present()  { return (readReg8(REG_STATUS0) & 0x04) != 0; }
bool BQ25792::powerGood()   { return (readReg8(REG_STATUS0) & 0x08) != 0; }

/* ===================== faults ===================== */
void BQ25792::readFaults(uint8_t& fault0, uint8_t& fault1)
{
    uint8_t f[2] = {0, 0};
    readReg(REG_FAULT0, f, 2);
    fault0 = f[0];
    fault1 = f[1];
}

bool BQ25792::hasFault()
{
    uint8_t f0, f1;
    readFaults(f0, f1);
    return f0 || f1;
}

void BQ25792::faultString(char* buf, size_t len)
{
    if (len) buf[0] = 0;
    uint8_t f0, f1;
    readFaults(f0, f1);
    static const char* n0[] = { "VAC1_OVP","VAC2_OVP","CONV_OCP","IBAT_OCP",
                                "IBUS_OCP","VBAT_OVP","VBUS_OVP","IBAT_REG" };
    static const char* n1[] = { 0,0,"TSHUT",0,"OTG_UVP","OTG_OVP","VSYS_OVP","VSYS_SHORT" };
    for (int i = 0; i < 8; i++)
        if ((f0 & (1 << i)) && strlen(buf) + 12 < len) { strcat(buf, n0[i]); strcat(buf, " "); }
    for (int i = 0; i < 8; i++)
        if ((f1 & (1 << i)) && n1[i] && strlen(buf) + 12 < len) { strcat(buf, n1[i]); strcat(buf, " "); }
}

/* ===================== limits ===================== */
void BQ25792::setChargeVoltage_mV(uint16_t mv)     { writeReg16(REG_VREG,   (mv / 10) & 0x07FF); }
void BQ25792::setChargeCurrent_mA(uint16_t ma)     { writeReg16(REG_ICHG,   (ma / 10) & 0x01FF); }
void BQ25792::setInputCurrentLimit_mA(uint16_t ma) { writeReg16(REG_IINDPM, (ma / 10) & 0x01FF); }

void BQ25792::setVINDPM_mV(uint16_t mv)
{
    if (mv < 3600) mv = 3600;   /* clamped low per datasheet */
    writeReg8(REG_VINDPM, (uint8_t)(mv / 100));
}

uint16_t BQ25792::getVINDPM_mV()
{
    return (uint16_t)readReg8(REG_VINDPM) * 100;
}

/* ===================== control ===================== */
void BQ25792::enableCharging(bool on) { setBits(REG_CHG_CTRL0, on ? 0x20 : 0x00, on ? 0x00 : 0x20); }
void BQ25792::setHIZ(bool on)         { setBits(REG_CHG_CTRL0, on ? 0x04 : 0x00, on ? 0x00 : 0x04); }
void BQ25792::disableWatchdog()       { setBits(REG_CHG_CTRL1, 0x00, 0x07); }   /* WATCHDOG[2:0]=0 */
void BQ25792::enableACDRV1(bool on)   { setBits(REG_CHG_CTRL4, on ? 0x40 : 0x00, on ? 0x00 : 0x40); }
void BQ25792::enableACDRV2(bool on)   { setBits(REG_CHG_CTRL4, on ? 0x80 : 0x00, on ? 0x00 : 0x80); }
void BQ25792::enableExtILIM(bool on)  { setBits(REG_CHG_CTRL5, on ? 0x02 : 0x00, on ? 0x00 : 0x02); }

void BQ25792::enterShipMode(bool immediate)
{
    /* SFET_PRESENT MUST be set first. It tells the charger an external ship
     * FET (Q204) exists; without it every SDRV_CTRL write is silently
     * discarded -- the field reads back 0 and nothing happens. Measured on
     * hardware 2026-08-07: this single bit was the difference between "ship
     * mode does nothing" and working. */
    setShipFETPresent(true);
    disableWatchdog();
    /* REG11 SDRV_CTRL[2:1]: 0=idle 1=shutdown 2=ship 3=system power reset.
     * SDRV_DLY[0]: 1 = do NOT add the 10 s delay. Ship (not shutdown) so the
     * QON button can wake the board again. Takes effect on write. */
    uint8_t v = readReg8(REG_CHG_CTRL2);
    v = (v & ~0x07) | (0x02 << 1) | (immediate ? 0x01 : 0x00);
    writeReg8(REG_CHG_CTRL2, v);
}

void BQ25792::setShipFETPresent(bool present)
{
    setBits(REG_CHG_CTRL5, present ? 0x80 : 0x00, present ? 0x00 : 0x80);
}

void BQ25792::systemPowerReset()
{
    setShipFETPresent(true);    /* same requirement as ship mode */
    disableWatchdog();
    uint8_t v = readReg8(REG_CHG_CTRL2);
    v = (v & ~0x07) | (0x03 << 1) | 0x01;
    writeReg8(REG_CHG_CTRL2, v);
}

void BQ25792::configureCharging(uint16_t ichg_ma, uint16_t iindpm_ma, uint16_t vreg_mv)
{
    disableWatchdog();          /* stop the WD from resetting our config every 40 s */
    setHIZ(false);              /* make sure the input isn't ignored */
    enableExtILIM(false);       /* make the IINDPM register the sole input limit.
                                 * Repo issue #3 suspected the ILIM_HIZ divider caps
                                 * at ~600 mA; bench probe drew 799 mA with the pin
                                 * limit enabled, so no clamp up to ~0.8 A - but
                                 * draws beyond that are unverified, so keep the
                                 * pin out of the equation. */
    if (vreg_mv)   setChargeVoltage_mV(vreg_mv);
    if (ichg_ma)   setChargeCurrent_mA(ichg_ma);
    if (iindpm_ma) setInputCurrentLimit_mA(iindpm_ma);
    enableCharging(true);
}
