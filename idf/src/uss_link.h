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
/* PROTO 3 (2026-09-18): added the RAW TOTALIZER block at 0x50 and grew the
 * readable window 0x40 -> 0x80. Purely ADDITIVE: the result block (0x04..0x2F)
 * and the link-health block (0x30..0x3F) are byte-for-byte unchanged and keep
 * their own CRCs, so a PROTO 2 master reading a PROTO 3 slave still works
 * exactly as before -- it simply never reads above 0x3F. A PROTO 3 master
 * reading a PROTO 2 slave gets zeros and a failing CRC at 0x50, which is why
 * the master must gate the raw path on PROTO_VER >= 3 and fall back to
 * VOL_ML. */
#define USS_LINK_PROTO      3

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
#define USS_REG_VOL_WRAPS   0x1D    /* u8  times VOL_ML wrapped (see below)   */
/* VOL_ML is u32 MILLILITRES and the slave accumulates in microlitres, so the
 * published value rolls over every 4,294,967 mL. Measured 2026-09-14: with the
 * dTOF zero-offset uncorrected the bench integrated 14.6 L/min of pure bias and
 * wrapped every ~4.9 h, silently -- it looked exactly like a module reset.
 * Reconstruct the true total as:
 *     total_mL = VOL_ML + (uint64_t) VOL_WRAPS * 4294967296ULL
 * The slave now accumulates in SIGNED 64-bit microlitres, so VOL_ML only rolls
 * at 2^32 mL = 4.29 million litres -- years at any real flow. VOL_WRAPS is
 * therefore normally 0 and exists so the total stays reconstructible anyway.
 * Reversals SUBTRACT (the header text below always claimed this; the code used
 * to reset the total to zero instead). While the signed accumulator is
 * negative, VOL_ML and VOL_WRAPS both read 0 and recover if flow returns.
 * Additive: 0x1D was reserved, so no PROTO bump. */
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
/* 0x28..0x2E reserved (0x00) */
/* -- capture-quality counters. Additive, no PROTO bump: 0x28..0x2E was
 * reserved space inside the CRC'd block. These restore PER-CAPTURE statistics,
 * which the master otherwise cannot see -- it samples one capture per record
 * while the slave runs ~300 between records. */
#define USS_REG_CAP_N       0x28    /* u16 total captures (wraps)             */
#define USS_REG_CAP_BADCODE 0x2A    /* u16 captures whose code != 122 (wraps) */
#define USS_REG_CAP_BADSNR  0x2C    /* u16 code==122 but SNR below threshold  */
#define USS_REG_RECOVERIES  0x2E    /* u8  abs-ToF re-searches forced by the
                                 * module (wraps; 0 means never fired)       */
#define USS_REG_CRC8        0x2F    /* u8  CRC8 over regs 0x04..0x2E          */
/* -- LINK HEALTH block. Additive, no PROTO bump, and guarded by its OWN CRC so
 * it can be read without disturbing the latched result block.
 *
 * This exists because the module's UART cannot be attached while it sits on the
 * comm board, and under ECOTRACE_LPM3 the UART cannot receive at all -- so the
 * only way to ask the module why it restarted is over I2C.
 *
 * STARTS is the useful one when a read fails: delta(STARTS) ~= the number of
 * transactions attempted means the bus reached the module and the fault is
 * above the physical layer; delta(STARTS) ~= 0 means the master never got on
 * the bus at all. RSTCAUSE carries the raw SYSRSTIV latched at boot
 * (0x0002 brownout, 0x000E SVSH, 0x0014 PMMSWPOR). */
#define USS_REG_LH_RSTCAUSE 0x30    /* u16 SYSRSTIV latched at boot           */
#define USS_REG_LH_STARTS   0x32    /* u16 I2C address matches (wraps)        */
#define USS_REG_LH_STOPS    0x34    /* u16 STOP conditions seen (wraps)       */
#define USS_REG_LH_RXBYTES  0x36    /* u16 bytes the master wrote (wraps)     */
#define USS_REG_LH_TXBYTES  0x38    /* u16 bytes the master read (wraps)      */
#define USS_REG_LH_CMDS     0x3A    /* u16 commands accepted (wraps)          */
#define USS_REG_LH_UPTIME_S 0x3C    /* u16 seconds since boot, saturates      */
#define USS_REG_LH_LASTCMD  0x3E    /* u8  last command byte received         */
#define USS_REG_LH_CRC8     0x3F    /* u8  CRC8 over regs 0x30..0x3E          */
#define USS_LH_OFF          0x30
#define USS_LH_LEN          0x10

/* -- RAW TOTALIZER block (PROTO 3). Own CRC; read independently of the result
 * block, so a master can poll the total without disturbing the latched sample.
 *
 * WHY THIS EXISTS. The module integrates at 1 Hz; the master wakes every ~5
 * minutes. Anything the master totalizes from its own snapshots samples flow at
 * 1/300 the rate and aliases. So integration MUST happen here -- but every
 * calibration constant then gets baked into MSP430 firmware, which needs a
 * physical MSP-FET to change. The ESP32 has OTA; the MSP430 does not.
 *
 * The way out: accumulate sums that contain NO calibration constants at all.
 * For a diagonal-path transit-time meter the flow is a pure geometric identity,
 * with no sound-speed and no gas-property term:
 *
 *     t_up = L/(c - v cos@),  t_dn = L/(c + v cos@)
 *     dtof / (t_up * t_dn)  =  2 v cos@ / L
 *     Q = A*v = [ A*L / (2 cos@) ] * dtof / (t_up * t_dn)
 *              \________________/
 *                  K, geometry only -- lives on the MASTER
 *
 * So the slave accumulates the integrand and the master owns K. It also
 * accumulates S0, the same integrand with dtof replaced by 1, which is what a
 * zero-flow dTOF offset WOULD have subtracted at full 1 Hz fidelity:
 *
 *     V_actual = K * ( dS1/2^24  -  offset_ps * dS0/2^40 )
 *     V_std    = V_actual * (P_mbar/1013.25) * (293.15/T_kelvin)
 *
 * Both K and offset_ps live on the master, are OTA-updatable, and can be
 * applied RETROACTIVELY -- re-zero a deployed unit months later and every past
 * window recomputes correctly, because S0 preserved the sensitivity. That is
 * why USS_CMD_ZEROCAL is no longer the only zeroing path, and why the master
 * should prefer deferred zeroing over burning an offset into FRAM.
 *
 * Both sums are SIGNED and LINEAR, so zero-flow noise cancels instead of
 * rectifying. (The VOL_ML path had to floor its accumulator at zero to stop
 * noise integrating into ~13 L/h of phantom flow; that hack is unnecessary
 * here.) Apply any low-flow cutoff on the master, per window, where it can be
 * retuned without a FET.
 *
 * S1 and S0 share one division, so their denominator rounding is IDENTICAL and
 * the offset subtraction cancels exactly at true zero flow. Do not recompute
 * either one independently. */
#define USS_REG_TOT_S1      0x50    /* i64 sum dtof_ps*dt_ms/(tu_ns*td_ns), Q24 */
#define USS_REG_TOT_S0      0x58    /* i64 sum      dt_ms/(tu_ns*td_ns), Q40    */
#define USS_REG_TOT_N       0x60    /* u32 valid samples accumulated            */
#define USS_REG_TOT_SKIP    0x64    /* u16 samples skipped (invalid) since reset*/
#define USS_REG_TOT_FLAGS   0x66    /* u8  bit0 = an accumulation saturated     */
#define USS_REG_TOT_CRC8    0x67    /* u8  CRC8 over 0x50..0x66                 */
#define USS_TOT_OFF         0x50
#define USS_TOT_LEN         0x18    /* 0x50..0x67 inclusive, CRC included       */
/* Fixed-point scaling. Defined HERE so both sides use one constant -- getting
 * these apart is the same class of trap as the Q40/Q20 ToF exponent. */
#define USS_TOT_S1_SHIFT    24
#define USS_TOT_S0_SHIFT    40      /* S0's integrand is ~1e5x smaller than S1's
                                     * at real flows, so it needs the extra 16
                                     * bits to avoid truncating to zero.       */
#define USS_TOT_FLAG_SAT    0x01

/* -- control -- */
#define USS_REG_CMD         0x40    /* u8  write-only, USS_CMD_*              */
#define USS_REG_AUTO_PERIOD 0x42    /* u16 autonomous measurement period, s
                                     *     (future; 0 = off)                  */

/* -- PARAMETER ACCESS (2026-09-25). Additive, no PROTO bump: an older master
 * never writes 0x44..0x48 and never reads 0x68..0x6F.
 *
 * WHY. Tuning the acoustic front end (gain, pulses, tones, window, dTOF offset)
 * used to need the bench image and the backchannel UART, which drops ~3 % of
 * characters, and every candidate setting meant a reflash. And the end state
 * this project wants -- the master detects gas composition from ToF and loads
 * the operating point that suits it -- needs exactly this write path anyway.
 *
 *   SET: write PARAM_ID (u8) and PARAM_VAL (i32 LE) in one transaction starting
 *        at 0x44, then CMD = USS_CMD_PARAM_SET. The main loop applies it
 *        between captures, reconfigures the library, and REVERTS on failure.
 *   GET: write PARAM_ID, then CMD = USS_CMD_PARAM_GET.
 *   Both: poll the PARAM result block below until PRM_SEQ changes and its CRC
 *        passes; PRM_VAL is the value NOW IN EFFECT (read back, not echoed).
 *
 * PERSISTENCE -- READ THIS. The whole USS configuration lives in #pragma
 * PERSISTENT FRAM, so a SET survives resets and power cycles until the next
 * flash. GET every parameter you intend to change BEFORE the first SET, and
 * write those values back to undo. The dTOF offset persists too (same store
 * as USS_CMD_ZEROCAL). */
#define USS_REG_PARAM_ID    0x44    /* u8  write-only, USS_PARAM_*            */
#define USS_REG_PARAM_VAL   0x45    /* i32 write-only, little-endian          */

/* Result block, own CRC, readable. */
#define USS_PRM_OFF         0x68
#define USS_PRM_LEN         0x08    /* 0x68..0x6F inclusive, CRC included     */
#define USS_REG_PRM_ID      0x68    /* u8  id the result belongs to           */
#define USS_REG_PRM_STAT    0x69    /* u8  USS_PRM_ST_*                       */
#define USS_REG_PRM_VAL     0x6A    /* i32 value in effect after the command  */
#define USS_REG_PRM_SEQ     0x6E    /* u8  +1 per completed PARAM command     */
#define USS_REG_PRM_CRC8    0x6F    /* u8  CRC8 over 0x68..0x6E               */

#define USS_PRM_ST_OK       0x00
#define USS_PRM_ST_UNKNOWN  0x01    /* no such id (or refused, e.g. AGC)      */
#define USS_PRM_ST_RANGE    0x02    /* value outside the allowed range        */
#define USS_PRM_ST_APPLY    0x03    /* library rejected it; old value restored */

/* Parameter ids. 0x00..0x0F are the firmware's own table (ecotrace_proto.c
 * PARAMS[], same order) -- change both together. Ranges are enforced there. */
#define USS_PARAM_GAIN        0x00  /* PGA index 17..63 (~0.85 dB/step)       */
#define USS_PARAM_AGC         0x01  /* REFUSED over I2C: the library AGC hangs
                                     * this board (DEVLOG 2026-08-13)         */
#define USS_PARAM_F1          0x02  /* Hz                                     */
#define USS_PARAM_F2          0x03  /* Hz                                     */
#define USS_PARAM_SAMPLES     0x04  /* ADC samples per capture                */
#define USS_PARAM_OVERSAMPLE  0x05
#define USS_PARAM_THRESH_HI   0x06
#define USS_PARAM_THRESH_LO   0x07
#define USS_PARAM_GAP_PPG     0x08  /* ASQ counts                             */
#define USS_PARAM_GAP_ADC     0x09  /* ASQ counts                             */
#define USS_PARAM_GAP_ADCSMP  0x0A  /* ASQ counts: excitation -> first sample */
#define USS_PARAM_CAP_TIMEOUT 0x0B
#define USS_PARAM_TOF_NEG     0x0C  /* abs-ToF search range, samples          */
#define USS_PARAM_TOF_POS     0x0D
#define USS_PARAM_DTOF_SHIFT  0x0E  /* maxSampleShift gate, samples           */
#define USS_PARAM_PULSES      0x0F  /* trill cycles (2 pulses each)           */
#define USS_PARAM_DTOF_OFF_PS 0x40  /* zero-flow dTOF offset the library
                                     * subtracts, ps, +-1000000               */

#define USS_LINK_RESULT_OFF 0x04
#define USS_LINK_RESULT_LEN 44      /* 0x04..0x2F inclusive, CRC included     */
#define USS_LINK_REGFILE_SIZE 0x40  /* the RAM-backed register file, 0x00..0x3F.
                                     * DELIBERATELY NOT GROWN for PROTO 3.
                                     * The MSP430FR6043 has 4 KB of RAM and the
                                     * build leaves ~150 bytes spare next to a
                                     * 512-byte stack that TI's capture code
                                     * runs on; doubling this array to carry the
                                     * totalizer would have eaten 64 of them.
                                     * The totalizer block is latched in FRAM
                                     * instead (8 KB spare, written at 1 Hz
                                     * against ~1e15-cycle endurance) and the
                                     * slave dispatches on the pointer. */
#define USS_LINK_ADDR_MASK    0x7F  /* the ADDRESS space the pointer wraps in,
                                     * which is wider than the register file
                                     * above. Anything inside the mask but
                                     * mapped to neither block reads 0, so an
                                     * over-long read still cannot walk RAM --
                                     * that guarantee now comes from the explicit
                                     * range checks in reg_read(), not from the
                                     * array size. USS_REG_CMD (0x40) and
                                     * USS_REG_AUTO_PERIOD (0x42) stay outside
                                     * the register file and so stay unreadable,
                                     * exactly as in PROTO 2.                  */

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
#define USS_CMD_VOL_RESET   0x05    /* zero the volume totalizer.
                                 * AUTO_START used to zero it implicitly, which
                                 * made the total useless the moment the master
                                 * power-cycled the module: every recovery threw
                                 * the reading away. The total now PERSISTS in
                                 * FRAM across module resets, so clearing it had
                                 * to become something you ask for.           */
#define USS_CMD_ZEROCAL     0x04    /* null the dTOF offset at the CURRENT flow.
                                 * ONLY valid at genuine zero flow -- it folds
                                 * whatever is being measured into the offset.
                                 * Needed because the deployment image has no
                                 * other calibration path: LPM3 stops SMCLK, so
                                 * the backchannel UART cannot receive, and the
                                 * offset is otherwise a compile-time constant
                                 * that a reset restores. Result lands in
                                 * DTOF_PS; persisted to FRAM.               */
#define USS_CMD_PARAM_SET   0x06    /* apply PARAM_ID/PARAM_VAL; see above     */
#define USS_CMD_PARAM_GET   0x07    /* read PARAM_ID back into the PRM block   */
#define USS_CMD_SOFT_RESET  0x0F

/* USS message codes we care about on the master side (from the USS library) */
#define USS_CODE_OK         122     /* valid measurement                      */
#define USS_CODE_NO_ECHO    126
/* Module-generated (not TI): a code-122 capture REJECTED because its abs-ToF
 * was implausible -- up and down disagree by more than a real dTOF can, or the
 * mean jumped away from the tracked value (the multitone lock landing one or
 * two 10.05 us pattern periods off). Treated exactly like any invalid capture:
 * no flow, counted as skipped in the raw totalizer, and it feeds the re-search.
 * 0xE0 is taken by the firmware's "rails off" code. */
#define USS_CODE_TOF_HOP    0xE1

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
