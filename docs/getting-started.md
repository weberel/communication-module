# Getting started

## 1. Install the toolchain

1. Install [VS Code](https://code.visualstudio.com/).
2. Install the **PlatformIO IDE** extension (this project recommends it in
   `.vscode/extensions.json`; VS Code will offer to install it when you open the
   folder).
3. Open this folder (`communication-module`) in VS Code. PlatformIO will download the
   pinned `pioarduino` platform on first build - this takes a few minutes once.

You do **not** need the Arduino IDE or a separate ESP-IDF install.

## 2. Connect the board

- Plug into the **USB-C** connector. The ESP32-C6 exposes a **native USB Serial/JTAG**
  interface (GPIO12/13) - no external USB-UART adapter needed for flashing or the
  serial monitor.
- **Keep a charged battery connected** while flashing. USB-only power can brown the
  board out during flashing (see [errata](hardware-errata.md#-3-brownout-when-flashing-without-a-battery)).

## 3. Build & flash

There are two build targets (PlatformIO environments):

```bash
pio run -e functionality_test -t upload && pio device monitor   # board self-test
pio run -e datalogger -t upload && pio device monitor           # the main app
```

Or use the PlatformIO toolbar / project tasks in VS Code and pick the env.

On a fresh board, run `functionality_test` first: it probes every device (charger,
sensors, flash, modem, WiFi) and prints a `PASS / FAIL / SKIP` summary. Once that is
clean, flash `datalogger`.

## 4. Serial console notes (ESP32-C6 USB-CDC)

The C6 has only USB Serial/JTAG, so `Serial` is the USB CDC. Two consequences baked
into every example:

- `build_flags` set `-DARDUINO_USB_MODE=1` and `-DARDUINO_USB_CDC_ON_BOOT=1`.
- Each sketch does a `delay(1500)` at the top of `setup()` so the host has time to
  re-enumerate the CDC port after a reset - otherwise the first prints are lost.

If the monitor shows nothing after flashing, close and reopen `pio device monitor`
(the port re-enumerates on reset).

## 5. Recovery: forcing the ROM bootloader (BOOT pad)

If USB flashing ever fails (e.g. a sketch that reconfigures USB pins), force the
built-in ROM bootloader using the pads on the **back** of the board:

1. Hold **BOOT** (GPIO9 / pad TP403) low.
2. Pulse **RESET** (EN).
3. Release BOOT.
4. Flash normally.

## 6. Cellular / secrets

Anything that uses the modem needs your SIM's APN:

```bash
cp lib/EcoTrace/secrets.example.h lib/EcoTrace/secrets.h
# edit secrets.h -> set SIM_APN (and POST_URL if you want to test an upload)
```

`secrets.h` is gitignored so credentials never get committed.

## 7. Where to go next

- Log your own sensor: edit `readSample()` / `LogRecord` / `buildJson()` in
  [`src/datalogger/main.cpp`](../src/datalogger/main.cpp). It is a plain starting
  point - change whatever you need.
- Configure the logger: [`src/datalogger/config.h`](../src/datalogger/config.h)
  (sample interval, upload cadence, charge profile, MPPT bounds).
- Browse the available driver functions: [`api-reference.md`](api-reference.md).
