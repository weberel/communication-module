# ecoTrace Communication Module

> ## 🚧 HARDWARE VALIDATED, APPLICATION NOT YET - read this
> As of the **2026-08-06/07** bring-up campaign the **board is fully validated on
> real hardware**: both charge inputs (USB and solar, incl. VINDPM/MPPT control),
> cellular, WiFi, all I2C sensors, the SPI flash, deep sleep, and the QON button
> (wake / OTA mode / ship-mode power-off). GPS is the only untested block.
> The `lib/EcoTrace` drivers underneath all of that ran on hardware too.
>
> What has **NOT been exercised end-to-end** is the **`datalogger` application
> itself**: the 5-minute duty cycle, the flash ring log across power loss, MPPT
> tracking over a real day, the twice-daily ThingsBoard upload with clock sync,
> and watchdog/crash recovery. It compiles and boots; treat it as a scaffold to
> validate, not as proven.
>
> Full breakdown, including the measured numbers: [`docs/testing-status.md`](docs/testing-status.md).

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
| Storage | GD25Q128 16 MB SPI NOR flash |
| Security | ATECC608B secure element |
| Inputs | USB-C (power + flashing), SIM slot, battery connector, VIN (solar/DC in) |
| Antennas | u.FL for LTE and GPS |

Board revision: **Rev A, 2026-04-23**. Full KiCad project, schematic PDF, board
renders, and JLCPCB fab files are in [`hardware/`](hardware/).

## What you get

**1. The datalogger** (`src/datalogger/`) - the main application. Every 5-minute
wake it:
- reads the full battery + charger state (BQ25792) and the on-board I2C sensors
  (LTR-303 light, SC7A20 accel),
- runs a solar-management pass: **MPPT** (fractional-Voc + perturb-&-observe on
  VINDPM) to maximise input power, per-day **harvest accounting**, and a
  **weather-adaptive charge target** - good weather caps charging at ~80 % SoC to
  age the LiPo slower, bad weather allows 100 % for reserve,
- appends one 64-byte record to a **ring log on the 16 MB SPI flash**
  (power-loss safe, ~2.5 years of capacity),
- twice a day drains the backlog to **ThingsBoard** in timestamped batches -
  cellular first, **WiFi as backup** - and syncs the wall clock from the network,
- deep-sleeps in between with everything powered down (modem rail, flash,
  sensors, charger ADC); the BQ25792 keeps charging autonomously.

It is a **hackable starting point** - fork it and change whatever you need. To log
your own sensor: extend `LogRecord` in [`record.h`](src/datalogger/record.h) (spare
bytes are reserved), read it in `readSample()` in
[`main.cpp`](src/datalogger/main.cpp), and send it in `recordValues()` in
[`uplink.cpp`](src/datalogger/uplink.cpp). Connect external sensors on the SENSOR
(I2C) or SPI header. Tune the duty cycle / charge profile / MPPT / weather
thresholds in [`config.h`](src/datalogger/config.h).

**2. The driver library** (`lib/EcoTrace/`) - ready-to-call functions for every part
on the board. Pull in only what you need. Full list: [`docs/api-reference.md`](docs/api-reference.md).

| Driver | Covers | Status |
|--------|--------|--------|
| `BQ25792` | charging (USB + solar), ADC, status, faults, MPPT/VINDPM | ✅ full, datasheet-accurate |
| `ModemA7672` | LTE power/AT, SIM, registration, GPRS, HTTP GET/POST, time | ✅ full (GPS = stub) |
| `LTR303` | ambient light (CH0/CH1, lux) - datalogger's example sensor | ✅ |
| `SC7A20` | accelerometer (X/Y/Z mg) | ✅ |
| `ExtFlash` | GD25Q128 read/erase/program, power-down | ✅ (full 16 MB) |
| `MS5837` | MS5837-02BA barometer (pressure, temperature, altitude) | ✅ |
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

# 5. reflash over WiFi later (bench convenience): press the QON button 3x
#    (board joins WiFi and blinks once per second), then:
pio run -e datalogger_ota -t upload
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
  datalogger/        the main application
    config.h           duty cycle, charge profile, MPPT, weather, upload tunables
    record.h           the 64-byte on-flash log record (+ CRC)
    flash_log.*        power-loss-safe ring log on the 16 MB SPI flash
    solar.*            MPPT + harvest tracking + weather-adaptive charge target
    uplink.*           ThingsBoard upload: cellular first, WiFi backup, clock sync
    main.cpp           the wake/sample/sleep cycle
  functionality_test/  board bring-up self-test
docs/              getting started, pinout, errata, api reference, testing status
hardware/          KiCad project + exports
```

## Related

- `functionality_test_arduino` - the original bring-up test this repo grew from.
- `bms_stove` - an ESP-IDF application on this hardware (remote battery/stove
  datalogger), a reference for a full deployment.
