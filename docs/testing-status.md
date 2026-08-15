# Testing status

What is proven on real hardware vs what is a new port that has not been run yet.
Read this before trusting any single feature.

## Bring-up campaign, 2026-08-06 / 07

Two units were taken through the full self-test: **unit A** `8C:FD:49:03:C4:10`
(clean board) and **unit B** `8C:FD:49:05:CA:AC` (the board behind repo issues
#4 and #5). Everything below was measured, not inferred.

**The whole board is now validated.** Every peripheral, both radios, both charge
inputs, the button, and deep sleep have run on real hardware. GPS remains the
only untested block (deliberately - see the errata).

| Verified | Detail |
|---|---|
| USB charging | fast-charge CC at the 500 mA SDP limit. The earlier "no charging" was a **cable with a broken VBUS wire** - the ~2.5 V seen on VUSB was phantom back-feed from the ESP's D+ through the D212 ESD array, not a sagging rail |
| Solar / VIN charging | outdoor test, panel Voc 6.72 V -> VBUS 6.09 V, fast-charge. The VINDPM mechanism the datalogger's MPPT depends on is proven |
| Input current limit | 799 mA drawn with the ILIM_HIZ pin limit *enabled*, so the ~600 mA clamp reported in issue #3 does not reproduce (0.8-2 A still untested) |
| Battery NTC / TS | unit B's mis-fitted resistor reworked; both units now read `TS: normal` |
| Modem UART ceiling | stock 10 k level-shifter pull-ups: 230400 clean, 460800 dead. After reworking **R405-R408 to 2.2 k** (unit B): **921600 clean** - the A7672's own maximum, since it rejects `AT+IPR=1843200`. ~90 KB/s, so a 1.2 MB OTA takes ~15 s |
| SPI flash | GD25Q128 (16 MB). Unit B's "dead" flash was **two unwetted lands under the ESP module** (IO8 = flash CS, IO18 = SCLK; IO9 between them fine) - the chip was healthy all along |
| QON button | short press wakes from deep sleep, 3 presses enter WiFi OTA mode, 3 s hold powers the board off via BQ ship mode, ~1 s hold powers it back on |
| MS5837-02BA barometer | on the I2C "hat" header: PROM CRC-4 valid, 967.8 mbar / 25.9 C / ~385 m ASL at Zurich |

### The one that cost the most time: `SFET_PRESENT`

BQ25792 ship mode, shutdown and system power reset **all silently do nothing**
unless `SFET_PRESENT` (REG14 bit 7) is set first to tell the charger an external
ship FET is fitted. The failure mode is deceptive: the I2C write is ACKed, every
other bit in the same register writes normally, and `SDRV_CTRL` simply reads back
0 with no error anywhere. Hours went into eliminating the watchdog, adapter
presence, the QON pin, the ADC, `EN_HIZ`, `REG_RST`, both delay settings and all
four field values before the bit was found (in the older `bq25792` Pico test
repo's register header). `BQ25792::enterShipMode()` and `systemPowerReset()` now
set it automatically.

### Diagnostics added to `functionality_test`

Built while chasing the faults above, and kept because they turn a vague failure
into a located one:

- **SPI**: MISO pull test (short vs open), `0xAB` deep-power-down wake before the
  JEDEC read, SI-SO loopback echo, header-jumper line-follow tests, GPIO
  pad-attachment sniffing, and an open-vs-clamped discriminator. Between them
  these distinguish a dead chip, a solder bridge, an open module land, and a
  chip merely asleep - none of which look different from "JEDEC = 0x000000".
- **Charger**: VAC1/VAC2 sense readings, an `EN_ACDRV1` read-back, an input
  qualification watch, the ILIM_HIZ probe, and a solar/MPPT sweep that measures
  panel Voc unloaded and then hill-climbs VINDPM to find the real maximum power
  point without knowing anything about the panel.
- **Modem**: a baud sweep that escalates 230400 -> 3686400 and always restores
  115200, so a level-shifter rework can be measured before and after. (Quirk: the
  first AT after `AT+IPR` always fails - send a throwaway command.)

### Per-unit build overrides

Unit B runs with a two-wire bypass for its open module lands - flash CS on GPIO1
(TP410 -> U602 pin 1) and SCLK on GPIO9 (header jumper CS<->SCLK):

```
pio run -e functionality_test -t upload \
    --build-flag="-DPIN_SPI_CS_FLASH=1 -DPIN_SPI_CLK=9"
```

Both pins default to the normal 8/18 for every other board.

## Proven on hardware

These have run on an actual ecoTrace Communication Module and worked. They were
validated in the two projects this repo is built from:

- **`functionality_test_arduino`** - the Arduino board self-test (the same code is in
  this repo as `functionality_test`).
- **`bms_stove`** - a full ESP-IDF datalogger deployed on this board.

Everything in the table below was re-confirmed during the 2026-08 campaign unless
noted otherwise.

| Capability | Proven | Where |
|------------|:------:|-------|
| QON button: wake / OTA mode / ship-mode power off | ✅ | `button_test`, 2026-08-07 |
| Solar input + VINDPM (MPPT) control | ✅ | `functionality_test`, 2026-08-07 |
| MS5837-02BA barometer on the I2C hat header | ✅ | `functionality_test`, 2026-08-07 |
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
| `datalogger` main loop | ✅ core proven 2026-08-07 | Wake/sample/sleep cycle, flash ring log (incl. recovery + backlog catch-up after a wrong-URL era), button wake/forced upload, and the ThingsBoard upload over WiFi all ran on unit A (`dl-2.9`). Still unproven: MPPT over a real solar day, weather/80 % switching, multi-day sleep budget. |
| WiFi backup uplink + SNTP | ✅ 2026-08-07 | Cellular failed gracefully ("no SIM / no network") -> WiFi fallback connected at -56 dBm and drained the backlog to eu.thingsboard.cloud. |
| Cellular upload (`ModemA7672` HTTP) | ✅ 2026-08-10 | **580 records drained over LTE in one session** (attach -65/-77 dBm) after a weekend offline in a basement -- the full store-and-forward design proven end-to-end. ThingsBoard Cloud intermittently answered 500 to the back-to-back burst; per-batch retry powered through, and `UPLOAD_BATCH_GAP_MS` pacing now avoids it. |
| WiFi OTA mode (3x button press) - **HTTP path** | ✅ 2026-08-07 | `curl -F "image=@firmware.bin" http://<ip>/update` on unit A over a PC mobile hotspot: upload, verify, ota_0 -> ota_1 slot switch and reboot all worked first try (reverse swap not yet exercised). No rollback: a broken pushed image means recovering over USB. |
| WiFi OTA mode - **espota/UDP path** (`datalogger_ota` env) | ❌ failed everywhere tried | espota's UDP invitation went unanswered in every tested configuration - across eth-iot (drops peer UDP), and even on a direct hotspot link with WiFi power-save disabled and ICMP+TCP healthy. Use the HTTP path. Bench findings: eth-iot passes ICMP between subnets but drops peer UDP; Windows mobile hotspot defaults to 5 GHz (C6 is 2.4-only) and auto-stops when no client connects; ESP modem power-save caused ~0.5 s RTT / 25 % loss until `WiFi.setSleep(false)`. |
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

---

## Ultrasonic link brought up, 2026-08-14/15

**The gas node works end to end.** The comm module reads the MSP430FR6043 over
I2C, logs a record and uploads it. Measured, not inferred:

| Capability | Proven | Evidence |
|---|:---:|---|
| I2C slave link, PROTO 2 | ✅ | `who_am_i 0x5A`, `proto 2`; 20/20 then 15/15 valid, CRC clean |
| Measurement over I2C, no UART | ✅ | `code=122`, 1301 ms mean / 1675 ms max vs a 3000 ms budget |
| Absolute ToF | ✅ | 131.3 us -> **335 m/s** on the 44 mm cell (air ~346) |
| Autonomous 1 Hz + totalizer | ✅ | `st=0x09 AUTO`, `vol_ml` accumulating |
| Slave watchdog (~3.2 s) | ✅ | 16 consecutive measurements, uptime monotonic, no spurious resets |
| All six I2C devices | ✅ | `sensor_ok=0x3F`; scan shows 7 incl. ATECC608 |
| Cellular upload with this payload | ✅ | "26 records sent, 0 pending" |

### Not proven / not calibrated

| Item | State |
|---|---|
| **Flow calibration (VFR constants)** | **Wrong for this cell.** `flow_lpm` reads several L/min at zero flow and `vol_ml` integrates it faithfully, so absolute volume is meaningless. Needs the lab DOE; no firmware change after. |
| Gas pressure | MS5837 is **not plumbed into the line yet** -- it reads ambient, so `dp_hpa` is not meaningful |
| Atmospheric pressure | a per-site **constant** (`P_ATM_CONST_HPA`), no sensor. Set before deployment: Nairobi at the Zurich value is ~18 % off |
| WF280A | **does not work.** Status byte 0x19 on every access = ADC powered off + test mode; a datasheet-correct trigger returns 6. Compensation polynomial is unpublished. Bosch part on the respin |
| GPS | still a stub |

### Bench gotchas for whoever picks this up

* **The ultrasonic board needs a power cycle after flashing.** SBW flashing
  succeeds while the CPU never starts. On the node its rail comes from this
  board, so it only bites on the bench.
* **Its `/SDA` trace is broken between J2 and R22** and is currently **bodged
  J2.4 -> R22.1**. Do not remove that wire. The layout routes `/SDA` through
  U8's pad, so one unwetted LGA land cuts the whole bus.
* **Feed its `/VCC` from VSYS, not 3V3** -- its onboard XC6206 needs headroom;
  3.3 V in leaves it in dropout.
* The USB CDC takes several seconds to enumerate after a wake, so a console
  attached mid-cycle misses the sampling logs entirely. A `USS recap` line is
  emitted just before deep sleep for exactly this reason.
