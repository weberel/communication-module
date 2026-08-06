/*
 * BQ25792.h  --  driver for the TI BQ25792 buck-boost charger / PMIC
 * =================================================================
 * The BQ25792 is the heart of the ecoTrace power path: it charges the LiPo from
 * USB (VBUS / VAC1) or solar (VAC2), regulates VSYS, and exposes a 16-bit ADC for
 * VBAT / VBUS / VSYS / IBAT / IBUS. There is NO fuel gauge -- state of charge must
 * be inferred from voltage + coulomb counting.
 *
 * Register addresses and scaling in this driver are taken directly from the TI
 * datasheet (SLUSDG1D, Apr 2026). Note for anyone comparing against the older
 * functionality-test code: that code wrote ICHG/ACDRV to the wrong offsets and
 * only worked because the reset defaults were already usable. This driver uses
 * the correct registers:
 *     VREG   = REG01/02 (10 mV/LSB)      ICHG   = REG03/04 (10 mA/LSB)
 *     VINDPM = REG05    (100 mV/LSB)     IINDPM = REG06/07 (10 mA/LSB)
 *     ACDRV1/2 = REG13[6]/[7]           EN_IBAT = REG14[5]
 *
 * All multi-byte registers are big-endian (MSB at the lower address).
 * Currents (IBAT, IBUS) are 16-bit two's-complement: + = into battery / into VBUS.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ecotrace_pins.h"

class BQ25792 {
public:
    enum ChgStat : uint8_t {
        NOT_CHARGING = 0, TRICKLE = 1, PRECHARGE = 2, FAST_CC = 3,
        TAPER_CV = 4, RESERVED = 5, TOPOFF = 6, DONE = 7
    };

    explicit BQ25792(TwoWire& bus = Wire, uint8_t addr = ECO_ADDR_BQ25792)
        : _bus(&bus), _addr(addr) {}

    /* True if a BQ25792 (part number 001) answers on the bus. */
    bool begin();
    bool isPresent();

    /* ---- ADC ---------------------------------------------------------------- */
    void enableADC(bool continuous = true);   /* ADC_EN, 15-bit resolution */
    void disableADC();
    void enableIbatSensing(bool on = true);   /* REG14[5]; needed for discharge (IBAT<0) */

    uint16_t readVbat_mV();
    uint16_t readVbus_mV();
    uint16_t readVsys_mV();
    uint16_t readVac1_mV();   /* USB input after boost */
    uint16_t readVac2_mV();   /* PV / solar input */
    int16_t  readIbat_mA();   /* + charge, - discharge (needs enableIbatSensing) */
    int16_t  readIbus_mA();   /* + into VBUS */

    /* ---- Status ------------------------------------------------------------- */
    ChgStat     chargeState();
    const char* chargeStateName();
    bool vbusPresent();   /* REG1B[0] */
    bool ac1Present();    /* REG1B[1] -- USB path */
    bool ac2Present();    /* REG1B[2] -- solar path */
    bool powerGood();     /* REG1B[3] */

    /* ---- Faults ------------------------------------------------------------- */
    void readFaults(uint8_t& fault0, uint8_t& fault1);
    bool hasFault();
    /* Writes a human-readable space-separated fault list into buf; "" if none. */
    void faultString(char* buf, size_t len);

    /* ---- Limits ------------------------------------------------------------- */
    void setChargeVoltage_mV(uint16_t mv);      /* VREG,   10 mV steps */
    void setChargeCurrent_mA(uint16_t ma);      /* ICHG,   10 mA steps */
    void setInputCurrentLimit_mA(uint16_t ma);  /* IINDPM, 10 mA steps */
    void     setVINDPM_mV(uint16_t mv);         /* input voltage limit, 100 mV steps */
    uint16_t getVINDPM_mV();

    /* ---- Control ------------------------------------------------------------ */
    void enableCharging(bool on = true);   /* REG0F[5] EN_CHG */
    void setHIZ(bool on);                  /* REG0F[2] EN_HIZ (ignore input, run off battery) */
    void disableWatchdog();                /* REG10[2:0]=0 so the charger keeps our settings */
    void enableACDRV1(bool on = true);     /* USB path gate (locked off if FET not populated) */
    void enableACDRV2(bool on = true);     /* solar path gate */
    void enableExtILIM(bool on = true);    /* ILIM_HIZ pin current clamp (REG14[1]) */

    /* Convenience: disable watchdog, clear HIZ, enable charging, apply limits.
     * Pass 0 for any limit you want left at its current value. */
    void configureCharging(uint16_t ichg_ma, uint16_t iindpm_ma, uint16_t vreg_mv = 0);

    /* ---- Raw register access (for experimentation / dumps) ------------------ */
    bool     readReg(uint8_t reg, uint8_t* buf, size_t len);
    uint8_t  readReg8(uint8_t reg);
    uint16_t readReg16(uint8_t reg);           /* big-endian */
    bool     writeReg8(uint8_t reg, uint8_t value);
    bool     writeReg16(uint8_t reg, uint16_t value);
    bool     setBits(uint8_t reg, uint8_t set, uint8_t clr);

    /* ---- Register map (public so examples can dump/experiment) -------------- */
    static constexpr uint8_t REG_VSYSMIN   = 0x00;
    static constexpr uint8_t REG_VREG      = 0x01;  /* 16-bit, 10 mV */
    static constexpr uint8_t REG_ICHG      = 0x03;  /* 16-bit, 10 mA */
    static constexpr uint8_t REG_VINDPM    = 0x05;  /* 8-bit, 100 mV */
    static constexpr uint8_t REG_IINDPM    = 0x06;  /* 16-bit, 10 mA */
    static constexpr uint8_t REG_TIMER     = 0x0E;
    static constexpr uint8_t REG_CHG_CTRL0 = 0x0F;  /* EN_CHG[5] EN_HIZ[2] EN_TERM[1] */
    static constexpr uint8_t REG_CHG_CTRL1 = 0x10;  /* WATCHDOG[2:0] */
    static constexpr uint8_t REG_CHG_CTRL3 = 0x12;
    static constexpr uint8_t REG_CHG_CTRL4 = 0x13;  /* EN_ACDRV2[7] EN_ACDRV1[6] */
    static constexpr uint8_t REG_CHG_CTRL5 = 0x14;  /* EN_IBAT[5] */
    static constexpr uint8_t REG_STATUS0   = 0x1B;
    static constexpr uint8_t REG_STATUS1   = 0x1C;  /* CHG_STAT[7:5] VBUS_STAT[4:1] */
    static constexpr uint8_t REG_FAULT0    = 0x20;
    static constexpr uint8_t REG_FAULT1    = 0x21;
    static constexpr uint8_t REG_ADC_CTRL  = 0x2E;  /* ADC_EN[7] */
    static constexpr uint8_t REG_IBUS_ADC  = 0x31;  /* s16, 1 mA */
    static constexpr uint8_t REG_IBAT_ADC  = 0x33;  /* s16, 1 mA */
    static constexpr uint8_t REG_VBUS_ADC  = 0x35;  /* u16, 1 mV */
    static constexpr uint8_t REG_VAC1_ADC  = 0x37;  /* u16, 1 mV */
    static constexpr uint8_t REG_VAC2_ADC  = 0x39;  /* u16, 1 mV */
    static constexpr uint8_t REG_VBAT_ADC  = 0x3B;  /* u16, 1 mV */
    static constexpr uint8_t REG_VSYS_ADC  = 0x3D;  /* u16, 1 mV */
    static constexpr uint8_t REG_PART_INFO = 0x48;

private:
    TwoWire* _bus;
    uint8_t  _addr;
};
