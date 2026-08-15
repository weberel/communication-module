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

## 🟡 6. GPIO14 sensor rail powers the SENSOR header, not the on-board sensors

Not a defect, just worth knowing: `GPIO14` drives a populated high-side switch
(active HIGH) for the **external sensor rail** on the SENSOR header. The on-board
SC7A20 + LTR-303 are wired directly to 3V3, **not** this rail, so cutting GPIO14 in
sleep does not power them down -- put those in low-power mode over I2C instead.

## 🔴 7. USB-C CC pins are floating - no VBUS (no charging) from C-to-C cables

`J201` (USB-C) has **CC1 and CC2 unconnected** in the schematic - there are no 5.1 k
Rd pulldowns. A spec-compliant USB-C source (laptop port, C-to-C cable, PD charger)
therefore never detects a sink and **never enables VBUS**: flashing and serial still
work (USB 2.0 data lines are hardwired and the ESP runs from the battery), but 0 V
reaches the charger. With a **USB-A to C cable** this erratum does not apply (A ports
supply 5 V unconditionally).

**Action:** for a respin, add 5.1 k from CC1 to GND and 5.1 k from CC2 to GND at
J201. Until then, charge via an A-to-C cable or VIN/solar.

**Debugging gotcha (resolved 2026-08-06, keep in mind):** if USB charging doesn't
start, don't trust a ~2.5 V reading on the `VUSB` net - that is a **phantom
voltage**, not a sagging 5 V rail. The ESP's D+ idles at 3.3 V and leaks through
the D212 (USBLC6) internal diode onto the VBUS net (3.3 V - 0.7 V, present even
with the cable unplugged). Since 2.6 V is below the BQ's ~3.6 V adapter-present
threshold, `AC1_PRESENT` stays 0 and the charger auto-clears `EN_ACDRV1`. On the
first Rev A unit the actual cause was a **cable with a broken VBUS wire** - the one
cable fault that is invisible on a self-powered board, because USB data (flashing,
serial) keeps working. With a good A-to-C cable the full path was verified: VAC1
~5.0 V, AC1_PRESENT=1, EN_ACDRV1 latches, fast-charge (CC) at the 500 mA SDP limit.
The self-test prints VAC1/VAC2 for exactly this diagnosis.

## 🟡 8. SPI flash is a GD25Q128 (16 MB), not the GD25Q256 on the schematic

The schematic symbol says GD25Q256 (32 MB), but the part populated on Rev A boards
reads JEDEC `C8 40 18` = **GD25Q128, 16 MB** (confirmed by the self-test). The
`ExtFlash` driver uses 3-byte addressing, which covers the whole 16 MB chip, so
nothing breaks -- just don't plan around 32 MB of storage.

---

### Notes on the old test firmware

The original `functionality_test_arduino` code charged the battery via the BQ25792 by
writing to **mislabelled registers** (it wrote ICHG to the IINDPM register and toggled
an ACDRV bit that is actually `WKUP_DLY`). It happened to work because the BQ's reset
defaults were already usable. The `BQ25792` driver in this repo uses the **correct**
registers per the TI datasheet (SLUSDG1D). If you port old snippets, cross-check the
register map in `lib/EcoTrace/BQ25792.h`.

---

## 🟠 9. Switched SENSOR rail has no bleed resistor

GPIO14's high-side switch leaves its output **floating** when it opens, so the
rail discharges only through whatever load is attached. Confirmed on the bench
2026-08-15 with an LED across the header: it **fades** rather than switching off.

Consequence: a short "power cycle" does not produce a power-on reset.
`board_sensor_power_cycle()` uses 800 ms for this reason.

**Respin:** ~100 kΩ from the switched rail to GND.

## 🔴 10. An unpowered device on the shared I2C bus takes the WHOLE bus down

Not specific to this board, but it bit hard and the symptom is misleading.

Any device on SDA/SCL that is unpowered while the bus is live will clamp it:
its ESD diodes hold the lines at ~0.6 V, and any pull-ups it has to *its* rail
become pull-downs to ground. I2C has no isolation, so **one clamped line makes
every device on those wires unreachable**.

Observed 2026-08-15 with the ultrasonic board unpowered: BQ25792 "not found",
light sensor and accelerometer silent, entire record zeros. All of them healthy.

Two error signatures separate this from a device fault, and they are worth
memorising:

* ESP-IDF **`clear bus failed`** = a line is held low -> hardware/power
* ESP-IDF **`unexpected nack`** = bus healthy, that device did not answer

**Respin:** if anything on a switched rail shares the bus, add an I2C isolator
(TCA9517/TCA4311A class) or two BSS138 FETs on SDA/SCL gated by the rail enable.
Otherwise "power-cycle the sensor" and "keep the charger readable" are mutually
exclusive -- and a node that cannot read its charger cannot manage its battery.

## 🟠 11. Cutting a device's VCC does not power it down while the bus idles high

The same coupling in reverse. Current flows: our 3V3 -> bus pull-up -> SDA ->
the device's ESD clamp -> its VCC. It sits about a diode drop below the bus:
**too low to run, too high to trigger a power-on reset**.

This is why `board_sensor_power_cycle()` holds SDA and SCL **low** for the whole
off window (`eco_i2c_hold_low()`), and why leaving a switched rail off during
deep sleep *costs* ~1.4 mA rather than saving anything.

## 🟡 12. Telemetry datapoint budget is a real constraint

ThingsBoard Cloud closed the MQTT connection (`transport_read(): EOF`,
`mqtt_message_receive() returned -2`) once records carried ~40 datapoints at
8 records per publish (~320 per message). Trimmed to 22 keys per record.

Related, and worse because it is silent: the record payload buffer was 800 bytes
and the JSON grew past it. `snprintf` truncated mid-field, the JSON became
malformed, and the server rejected **the whole batch** with no error anywhere.
The symptom was every sensor key frozen at one timestamp while the separate,
shorter status payload kept updating. `record_values()` now returns -1 on
truncation so it cannot fail quietly again.

Also: keep large payload buffers **static**. A 1536-byte automatic on the 4 kB
main task stack triggered `Guru Meditation Error: Core 0 panic'ed (Stack
protection fault)` mid-upload, rebooting the device every cycle.
