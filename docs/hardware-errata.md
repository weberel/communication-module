# Hardware errata & gotchas - ecoTrace Communication Module Rev A (2026-04-23)

Read this before wiring or deploying. These are known issues on the current boards.

## 🔴 1. VIN +/- silkscreen is SWAPPED

The `+` and `-` labels next to the **VIN** screw terminal (top of the board) are
**reversed**. The terminal that is silk-labelled `+` is actually the negative, and
vice versa. This is on purpose noted very visibly because it is easy to get wrong.

**Action:** ignore the silkscreen. Verify polarity against the schematic / with a
meter before connecting a panel or DC source. Reversed input can damage the board.

## 🔴 2. Battery NTC (TS) input - wrong resistor value as shipped

The boards were populated with an **incorrect resistor value at the battery NTC (TS)
input** of the BQ25792. Left unfixed, the charger may misread battery temperature and
refuse to charge / report false TS_COLD/TS_HOT faults.

**Action:** this should be **fixed (reworked) on all boards** before use. Confirm the
rework was done on your unit. If charging never starts and the BQ reports TS faults,
suspect this first.

## 🟠 3. Brownout when flashing without a battery

Flashing / heavy activity **without a battery connected** (USB only) can brown the
board out - not thoroughly characterised yet. The USB input alone may not hold VSYS
under inrush.

**Action:** keep a charged battery connected while flashing and during first bring-up.
If you see boot loops or resets only on USB-only power, this is the likely cause.

## 🟠 4. GPS is untested

The A7672E-LASE has an internal GNSS engine, but **GPS has not been validated** on
this board and the **GPS antenna path is unverified**. The `ModemA7672::gps*()`
methods are deliberately stubs.

**Action:** treat GPS as unproven. Validate the antenna path and AT+CGNSS* flow on
the bench before relying on location data.

## 🟡 5. Important pads are on the BACK of the board

Several essential test/programming pads are on the **bottom side**, including **BOOT**
(GPIO9), the UART0 flash pads (TXD0/RXD0), QON (GPIO2), and reset (EN). If flashing
over USB Serial/JTAG ever fails, you recover via the **BOOT pad + reset** (hold BOOT
low, pulse reset, release BOOT) to force the ROM bootloader. See
[`pinout.md`](pinout.md) for the pad list.

## 🟡 6. GPIO14 sensor-rail population varies

`GPIO14` drives a high-side switch for an **external sensor rail** (active HIGH). On
some boards this footprint is **not populated**; the on-board SC7A20 + LTR-303 are
always wired to 3V3 regardless and can only be low-powered over I2C. Confirm on your
board before relying on switching sensor power in sleep.

---

### Notes on the old test firmware

The original `functionality_test_arduino` code charged the battery via the BQ25792 by
writing to **mislabelled registers** (it wrote ICHG to the IINDPM register and toggled
an ACDRV bit that is actually `WKUP_DLY`). It happened to work because the BQ's reset
defaults were already usable. The `BQ25792` driver in this repo uses the **correct**
registers per the TI datasheet (SLUSDG1D). If you port old snippets, cross-check the
register map in `lib/EcoTrace/BQ25792.h`.
