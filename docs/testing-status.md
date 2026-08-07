# Testing status

What is proven on real hardware vs what is a new port that has not been run yet.
Read this before trusting any single feature.

## Proven on hardware

These have run on an actual ecoTrace Communication Module and worked. They were
validated in the two projects this repo is built from:

- **`functionality_test_arduino`** - the Arduino board self-test (the same code is in
  this repo as `functionality_test`).
- **`bms_stove`** - a full ESP-IDF datalogger deployed on this board.

| Capability | Proven | Where |
|------------|:------:|-------|
| Board bring-up: I2C scan, device IDs on the bus | ✅ | functionality_test |
| BQ25792 read (VBAT/VBUS/IBAT, charge state, faults) | ✅ | functionality_test, bms_stove |
| USB charging | ✅ | bms_stove |
| Solar charging + software MPPT (VINDPM tracking) | ✅ | bms_stove |
| Cellular: modem power, SIM, registration, HTTP upload | ✅ | bms_stove, functionality_test |
| Deep sleep + timed wake, RTC-RAM persistence | ✅ | bms_stove |
| GPIO14 external sensor power switch | ✅ | confirmed on hardware |
| LTR-303 light + SC7A20 accelerometer read | ✅ | functionality_test |
| GD25Q128 external flash (JEDEC / read-write) | ✅ | functionality_test |
| ATECC608B secure element presence/wake | ✅ | functionality_test |

The **hardware and every technique above are known good.** The board works.

## NOT run on hardware yet (this repo's Arduino rewrite)

The `datalogger` app and the `lib/EcoTrace` driver library are a fresh Arduino
implementation of the proven techniques above. They compile cleanly for both build
environments but have **not been exercised on a board**.

| Item | Status | Note |
|------|--------|------|
| `BQ25792` driver - charging | ⚠️ unrun | **Highest risk.** Uses the *correct* TI registers, which differ from the older (proven) code that charged via mislabelled ones. Charging as written here has never run. |
| `ModemA7672` driver - HTTP upload | ⚠️ unrun | Fresh Arduino AT port. The technique is proven (bms_stove uploaded), this exact re-implementation is not. |
| `datalogger` main loop | ⚠️ unrun | 5-min wake/sample/sleep cycle, SPI-flash ring log (`flash_log`), fractional-Voc + P&O MPPT and weather-adaptive 80 % charge target (`solar`), twice-daily ThingsBoard upload with WiFi backup and clock sync (`uplink`). None of it has run on a board. |
| WiFi backup uplink + SNTP | ⚠️ unrun | First WiFi use on this board at all -- nothing prior used the C6 radio. |
| WiFi OTA mode (3x button press, `datalogger_ota` env) | ⚠️ unrun | ArduinoOTA push into dual ota_0/ota_1 slots (new `partitions.csv`). No rollback: a broken pushed image means recovering over USB. |
| `LTR303` / `SC7A20` / `ExtFlash` / `ATECC608B` drivers | ⚠️ unrun | Thin re-ports of proven register sequences. Low risk, still unverified. |
| GPS (`ModemA7672::gps*`) | ❌ stub | Not implemented; GPS antenna path unvalidated on this board. |

`functionality_test` in this repo is a near-verbatim copy of the proven
`functionality_test_arduino` (hardcoded credentials were moved to `secrets.h`, and
the charge "nudge" was fixed to drive EN_ACDRV1 at REG13[6] -- the old copy wrote
the mislabelled REG12[3]/WKUP_DLY and gated on presence bits that stay 0 while
ACFET1 is off, so it never actually opened the USB input gate).

Update 2026-08-07: the **Rev A solar input path** (VAC2 -> ACFET2 -> VBUS, with
VINDPM control) was verified on hardware by `functionality_test`'s new solar/MPPT
sweep, outdoors with a real panel (Voc 6.7 V -> fast-charge CC from the panel). The
VINDPM mechanism the datalogger's `solar.cpp` MPPT relies on is therefore proven;
the datalogger code itself remains unrun.

## Bottom line

- **Flash `functionality_test` first** on any board - it is trusted and confirms the
  board is healthy.
- Treat the **`datalogger` as a working scaffold to validate**, not as known-good.
  When you first run it, watch the charge path: `chg_stat` should reach fast-charge
  and `IBAT` should go positive on USB/solar. Then confirm a record reaches your
  upload endpoint with a sane timestamp.
- See [`hardware-errata.md`](hardware-errata.md) for the physical gotchas (swapped VIN
  silk, NTC resistor rework, keep a battery connected when flashing).
