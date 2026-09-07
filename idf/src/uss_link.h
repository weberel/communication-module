/*
 * uss_link.h  --  I2C contract between the communication module (ESP32-C6,
 * bus MASTER) and the ultrasonic flow module (MSP430FR6043, eUSCI_B0 SLAVE).
 *
 * >>> This file is compiled on BOTH sides. The canonical copy lives in the
 * >>> communication-module repo (idf/src/uss_link.h); the ultrasonic repo
 * >>> carries an identical copy (Firmware/fw/uss_link.h). Change them together.
 *
 * Architecture (agreed 2026-08-13): ONE shared bus, ONE master.
 *   - The comm PCB I2C bus (GPIO6/7, 100 kHz) extends over the harness to the
 *     ultrasonic PCB's sensor bus on P1.6/P1.7 (broken out on J2/J3).
 *   - The MSP430 is an eUSCI_B0 I2C *slave* at USS_LINK_ADDR7. Its old
 *     bit-banged master (i2c.c) must be disabled in the linked build -- the
 *     ESP32 reads the WF280A (0x38) directly as just another slave.
 *   - Low power: the MSP430 sleeps in LPM3; eUSCI address match wakes it.
 *     It may clock-stretch while busy -- the master uses generous timeouts.
 *
 * Wiring/power caveat: the shared bus couples the two boards' power domains.
 * An unpowered ultrasonic PCB drags SDA/SCL low through its pull-ups and ESD
 * clamps and kills the *whole* sensor bus. Feed the ultrasonic /VCC (J2) from
 * the comm PCB's always-on VSYS/battery rail; idle cost is ~2-3 uA
 * (XC6206 quiescent + MSP430 LPM3). Both rails are 3V3-class: levels match.
 * Two pull-up pairs end up in parallel (comm PCB + R21/R22 4k7 = ~2k2
 * effective, ~1.5 mA sink) -- in spec at 100 kHz; DNP one pair if marginal.
 *
 * Transaction model (register-pointer, auto-increment, all multi-byte LE --
 * both MCUs are little-endian):
 *   write:  START addr+W [reg_ptr] [data ...] STOP
 *   read:   START addr+W [reg_ptr] RESTART addr+R [data ...] STOP
 *
 * Measurement handshake:
 *   1. master writes USS_CMD_MEASURE to USS_REG_CMD
 *   2. slave sets STATUS.BUSY, clears STATUS.READY, wakes, runs one USS
 *      measurement (typ. <500 ms; the slave may mask its I2C IRQ around the
 *      timing-critical capture -> polls stretch, never corrupt)
 *   3. slave latches the result block (0x04..0x1F) with CRC8, sets READY
 *   4. master polls STATUS, then reads the whole block in one transaction
 *      and verifies the CRC
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define USS_LINK_ADDR7      0x2C    /* free on the comm bus: 0x18/19 SC7A20,
                                     * 0x29 LTR303, 0x38/0x78 WF280A,
                                     * 0x60 ATECC608B, 0x6B BQ25792, 0x76 MS5837 */
#define USS_LINK_WHOAMI     0x5A
/* PROTO 2 (2026-08-14): result block grew 28 -> 44 bytes to carry ABSOLUTE
 * time-of-flight. The CRC moved 0x1F -> 0x2F, so a v1 master reading a v2 slave
 * would checksum the wrong offset and fail every sample -- which is the point of
 * the version byte. Reflash BOTH sides together. */
#define USS_LINK_PROTO      2

/* ---- register map ---- */
#define USS_REG_WHO_AM_I    0x00    /* u8  = USS_LINK_WHOAMI                  */
#define USS_REG_PROTO_VER   0x01    /* u8  = USS_LINK_PROTO                   */
#define USS_REG_FW_MAJ      0x02    /* u8  firmware version major             */
#define USS_REG_FW_MIN      0x03    /* u8  firmware version minor             */
/* -- result block: latched atomically by the slave, guarded by CRC8 -- */
#define USS_REG_STATUS      0x04    /* u8  USS_ST_* bits                      */
#define USS_REG_CODE        0x05    /* u8  USS message code; 122 = valid      */
#define USS_REG_SEQ         0x06    /* u16 slave measurement counter          */
#define USS_REG_FLOW_ULPM   0x08    /* i32 calibrated flow, uL/min            */
#define USS_REG_DTOF_PS     0x0C    /* i32 delta time-of-flight, picoseconds  */
#define USS_REG_TEMP_CC     0x10    /* i16 gas temperature, 0.01 C            */
#define USS_REG_AMP_UPS     0x12    /* u16 peak echo amplitude, upstream      */
#define USS_REG_AMP_DNS     0x14    /* u16 peak echo amplitude, downstream    */
#define USS_REG_SNR_DB2     0x16    /* u8  SNR in dB x2 (0..127.5 dB)         */
#define USS_REG_GAIN        0x17    /* u8  PGA gain index actually used       */
#define USS_REG_VOL_ML      0x18    /* u32 totalized volume, mL (autonomous
                                     *     mode, future -- 0 until implemented) */
#define USS_REG_VARIANT     0x1C    /* u8  USS_VARIANT_*: which medium the slave
                                     *     image was built for. Gas and water
                                     *     are separate builds (different USS
                                     *     optimized library, tone config and
                                     *     AFE routing), so the master cannot
                                     *     infer it. ADDED 2026-08-14 into space
                                     *     that was reserved-zero INSIDE the CRC
                                     *     block, so it is purely additive: a
                                     *     master that ignores it still reads a
                                     *     CRC-valid block. */
/* 0x1D..0x1F reserved (0x00) */
/* -- absolute time-of-flight, ADDED in PROTO 2 --
 * Sent as the library's RAW Q40 seconds, deliberately NOT converted to ps.
 * Absolute ToF is the speed-of-sound observable (gas composition), so the wire
 * carries exactly what the USS library produced and the scaling is applied
 * off-device. It also keeps a wrong exponent from being baked into two
 * firmwares: getting this wrong is a documented trap (assuming 2^20 instead of
 * 2^40 inflates ToF by ~4.9% and looks like a capture-window bug).
 *   microseconds = raw * 1e6 / 2^40   (= raw / 1099511.627776)             */
#define USS_REG_TOF_UPS_Q40 0x20    /* u32 absolute ToF upstream,   Q40 s     */
#define USS_REG_TOF_DNS_Q40 0x24    /* u32 absolute ToF downstream, Q40 s     */
/* -- capture-quality counters, ADDED 2026-09-06 (additive, no PROTO bump) --
 * Free-running u16 counts of every capture the module has run since ITS last
 * reset. They WRAP; the master takes the difference between consecutive reads,
 * which is correct modulo 65536 and needs no read-clear handshake (read-clear
 * would race the auto-incrementing register pointer during a block read).
 *
 * Why they exist: the master samples one capture per 5 minutes while the module
 * runs ~300, so a single uss_code is a coin flip, not a rate. On 2026-09-06 the
 * real bad-sample rate was ~30 % and two thirds of it was INVISIBLE -- those
 * captures reported code 122 "valid" with dtof 100-1000x out of family, and
 * only SNR gave them away. These two counters make the rate observable in one
 * telemetry cycle instead of an afternoon of guessing.
 *
 * A module that predates them answers 0 in all three, which is a clean
 * "not implemented" sentinel: a running module always has CAP_N != 0.
 * Deliberately NOT a PROTO bump -- they land in space that was already
 * reserved and CRC-covered, so an old master still reads a CRC-valid block. */
#define USS_REG_CAP_N       0x28    /* u16 total captures (wraps)             */
#define USS_REG_CAP_BADCODE 0x2A    /* u16 captures whose code != 122 (wraps) */
#define USS_REG_CAP_BADSNR  0x2C    /* u16 code==122 but SNR below threshold;
                                     * the silent failures (wraps)            */
/* 0x2E reserved (0x00) */
#define USS_REG_CRC8        0x2F    /* u8  CRC8 over regs 0x04..0x2E          */
/* -- control -- */
#define USS_REG_CMD         0x40    /* u8  write-only, USS_CMD_*              */
#define USS_REG_AUTO_PERIOD 0x42    /* u16 autonomous measurement period, s
                                     *     (future; 0 = off)                  */

#define USS_LINK_RESULT_OFF 0x04
#define USS_LINK_RESULT_LEN 44      /* 0x04..0x2F inclusive, CRC included     */
#define USS_LINK_REGFILE_SIZE 0x40  /* readable window 0x00..0x3F; MUST stay a
                                     * power of two (the slave masks the
                                     * auto-increment pointer with SIZE-1 so an
                                     * over-long read can never walk RAM).
                                     * USS_REG_CMD (0x40) sits just outside it,
                                     * which is fine: it is write-only.        */

/* STATUS bits */
#define USS_ST_READY        0x01    /* result block valid & CRC'd             */
#define USS_ST_BUSY         0x02    /* measurement in progress                */
#define USS_ST_ERR          0x04    /* last measurement failed to run at all
                                     * (USS init/config error; CODE has detail) */
#define USS_ST_AUTO         0x08    /* autonomous totalizer running (future)  */
#define USS_ST_BOOT         0x80    /* set from reset until the first command:
                                     * lets the master detect a slave reboot
                                     * (and a lost totalizer)                 */

/* USS_REG_VARIANT values */
#define USS_VARIANT_UNKNOWN 0x00
#define USS_VARIANT_GAS     0x01    /* multi-tone 170/240 kHz, external AFE    */
#define USS_VARIANT_LIQUID  0x02    /* single-tone 1 MHz, direct drive         */

/* CMD values */
#define USS_CMD_MEASURE     0x01
#define USS_CMD_AUTO_START  0x02    /* future */
#define USS_CMD_AUTO_STOP   0x03    /* future */
#define USS_CMD_SOFT_RESET  0x0F

/* USS message codes we care about on the master side (from the USS library) */
#define USS_CODE_OK         122     /* valid measurement                      */
#define USS_CODE_NO_ECHO    126

/* Unit conversions (slave side):
 *   flow  : USS VFR float L/min      -> i32 uL/min : (int32_t)(flow * 1e6f)
 *   dtof  : Q44 fixed-point seconds  -> i32 ps     : dtof_q44 * 1e12 / 2^44
 *           (= dtof_q44 / 17.592186; do it in float or 64-bit)
 *   temp  : float C                  -> i16 cC     : (int16_t)(t * 100)
 *   snr   : float dB                 -> u8         : (uint8_t)(snr * 2 + 0.5) */

/* CRC8, poly 0x07, init 0xFF (nonzero init so an all-zero block can't pass).
 * Tiny and identical on both compilers. */
static inline uint8_t uss_link_crc8(const uint8_t *p, size_t n)
{
    uint8_t crc = 0xFF;
    size_t i;
    for (i = 0; i < n; i++) {
        int b;
        crc ^= p[i];
        for (b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1));
    }
    return crc;
}
