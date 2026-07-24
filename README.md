# ecoTrace Communication Module

> ## 🚧 NOT TESTED YET
> The **PCB has been tested** and works. **This firmware/repo has NOT** - it compiles
> cleanly but has not been run or verified on hardware. Treat everything here as a
> starting point to validate yourself, not as known-good. Expect bugs; check behaviour
> against the schematic and the datasheets before relying on it.

Firmware and board-support for the **ecoTrace Communication Module** - a small,
battery- and solar-powered ESP32-C6 board with an LTE modem, built for **logging
data remotely** from the field.

This repo gives you a ready-to-run **datalogger** plus a library of drop-in
**functions for every peripheral** on the board, so you can fork the logger and add
whatever you need. It is the starting point for anyone (students, collaborators)
picking up the board.

> ⚠️ **Read [`docs/hardware-errata.md`](docs/hardware-errata.md) before wiring anything.**
> Rev A has a **swapped VIN +/- silkscreen** and other gotchas.

---

## The board at a glance

| | |
|---|---|
| MCU | ESP32-C6-MINI-1 (WiFi / BLE / 802.15.4, native USB Serial/JTAG) |
| Power / charging | **BQ25792** buck-boost charger + PMIC - USB, solar (MPPT), LiPo, 16-bit ADC |
| Cellular | **SIMCom A7672E-LASE** LTE Cat-1 modem (+ internal GNSS, untested) |
| Sensors (optional) | SC7A20 accelerometer, LTR-303ALS ambient light |
| Storage | GD25Q256 32 MB SPI NOR flash |
| Security | ATECC608B secure element |
| Inputs | USB-C (power + flashing), SIM slot, battery connector, VIN (solar/DC in) |
| Antennas | u.FL for LTE and GPS |

Board revision: **Rev A, 2026-04-23**. Full KiCad project, schematic PDF, board
renders, and JLCPCB fab files are in [`hardware/`](hardware/).

## What you get

**1. The datalogger** (`src/datalogger/`) - the main application. Every cycle it:
- reads battery + charger state (BQ25792) and **your sensor**,
- keeps the battery charging from USB or solar (software MPPT for solar),
- buffers the sample in RTC RAM,
- uploads the buffer over cellular every N samples (HTTP POST),
- deep-sleeps in between.

It is a **plain, hackable starting point** - fork it and change whatever you need.
It logs the on-board LTR-303 light sensor as an example; to log your own sensor,
change `readSample()` (read it), the `LogRecord` struct (store it), and `buildJson()`
(send it) in [`src/datalogger/main.cpp`](src/datalogger/main.cpp). Connect external
sensors on the SENSOR (I2C) or SPI header. Tune the duty cycle / charge profile in
[`config.h`](src/datalogger/config.h).

**2. The driver library** (`lib/EcoTrace/`) - ready-to-call functions for every part
on the board. Pull in only what you need. Full list: [`docs/api-reference.md`](docs/api-reference.md).

| Driver | Covers | Status |
|--------|--------|--------|
| `BQ25792` | charging (USB + solar), ADC, status, faults, MPPT/VINDPM | ✅ full, datasheet-accurate |
| `ModemA7672` | LTE power/AT, SIM, registration, GPRS, HTTP GET/POST, time | ✅ full (GPS = stub) |
| `LTR303` | ambient light (CH0/CH1, lux) - datalogger's example sensor | ✅ |
| `SC7A20` | accelerometer (X/Y/Z mg) | ✅ |
| `ExtFlash` | GD25Q256 read/erase/program, power-down | ✅ (low 16 MB) |
| `ATECC608B` | secure element presence/wake | 🔹 thin (crypto via CryptoAuthLib) |
| `EcoTraceBoard` | safe init, I2C/SPI bring-up, deep sleep | ✅ |

**3. The board self-test** (`src/functionality_test/`) - probes every device and
prints a `PASS / FAIL / SKIP` report. Run it first on a new board.

## Toolchain

**VS Code + PlatformIO + Arduino-ESP32.** Reproducible builds, a real editor, the
Arduino library ecosystem, and full ESP-IDF APIs underneath when needed. See
[`docs/getting-started.md`](docs/getting-started.md).

The ESP32-C6 needs arduino-esp32 v3+, which the official PlatformIO platform does not
ship yet - this project pins the [pioarduino](https://github.com/pioarduino/platform-espressif32)
fork (version `53.03.13`) in `platformio.ini`. Don't unpin it.

## Quick start

```bash
# 1. open this folder in VS Code with the PlatformIO extension installed
# 2. plug the board in over USB-C (keep a battery connected -- see errata)
# 3. check the board is healthy:
pio run -e functionality_test -t upload && pio device monitor

# 4. set your SIM APN + upload URL, then run the logger:
cp lib/EcoTrace/secrets.example.h lib/EcoTrace/secrets.h   # then edit it
pio run -e datalogger -t upload && pio device monitor
```

Without a `POST_URL` in `secrets.h` the datalogger runs in bench mode: it samples and
prints over serial but does not transmit.

## Layout

```
lib/EcoTrace/      board support: pin map + drivers (the reusable core)
  ecotrace_pins.h    single source of truth for the pin map
  EcoTraceBoard.*    safe init, I2C/SPI bring-up, deep-sleep helper
  BQ25792.*          charger / PMIC (hand-written, datasheet-accurate)
  ModemA7672.*       LTE modem (hand-written AT stack)
  LTR303.* SC7A20.* ATECC608B.* ExtFlash.*   peripheral drivers
  secrets.example.h  copy to secrets.h and fill in (gitignored)
src/
  datalogger/        the main application (+ config.h)
  functionality_test/  board bring-up self-test
docs/              getting started, pinout, errata, api reference, power states
hardware/          KiCad project + exports
```

## Related

- `functionality_test_arduino` - the original bring-up test this repo grew from.
- `bms_stove` - an ESP-IDF application on this hardware (remote battery/stove
  datalogger), a reference for a full deployment.
