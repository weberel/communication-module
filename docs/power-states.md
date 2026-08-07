# Power states

The board's intended power/charging states are documented in the diagram
[`ecoTrace_power_states.pdf`](ecoTrace_power_states.pdf) in this folder.

Key ideas (see the PDF for the authoritative version):

- The **BQ25792** is the PMIC: it manages input selection (USB / VBUS via VAC1, solar
  via VAC2), battery charging, and the regulated system rail **VSYS** that powers the
  ESP32-C6 and peripherals.
- Deep-sleep duty cycling (see `src/datalogger/`) is how the logger keeps average
  current low: the ESP32 sleeps most of the time, waking to sample and - less often -
  to power the modem and transmit.
- The modem rail (GPIO23) and external sensor rail (GPIO14) are switched off and held
  off during sleep so they don't dominate the sleep budget.

## The QON button (power/wake)

One button drives the shared QON net (BQ25792 QON input + ESP GPIO2, external
pull-up):

| Action | Effect | Implemented by |
|---|---|---|
| short press (asleep) | wakes the ESP for an immediate sample | EXT1 deep-sleep wake on GPIO2 |
| 3 presses in ~2.5 s | WiFi **OTA mode** for 5 min (`pio run -e datalogger_ota -t upload`) | firmware (`ota.cpp`) |
| hold ~3 s, then release | **power off**: BQ ship mode, battery disconnected, ~129 µA | firmware (`BQ25792::enterShipMode`) |
| hold ~1 s (while off) | **power on**: BQ exits ship mode, board cold-boots | BQ hardware (tSM_EXIT) |
| plug in USB or solar (while off) | also exits ship mode - a shipped board revives itself in the field | BQ hardware, not disableable |
| hold ~10 s (any time) | full hardware power cycle (ship FET off 350 ms) - the unbrick reset | BQ hardware (tRST), cannot be disabled |

> **`SFET_PRESENT` (REG14 bit 7) must be set before any `SDRV_CTRL` action.** It
> tells the charger that an external ship FET (Q204) is fitted. Without it every
> ship-mode / shutdown / system-power-reset write is silently discarded: the
> write is ACKed, the field reads back 0, and nothing happens. This cost a full
> debugging session on 2026-08-07; `BQ25792::enterShipMode()` and
> `systemPowerReset()` now set it automatically.

Ship mode entry requires **no input present** - with USB or solar connected the
charger refuses (documented), so the board only powers down once unplugged. The
button never enters ship mode while it is still held down, because QON low for
~1 s is the charger's wake-from-ship signal.

Notes: ship mode disconnects only the battery - with USB/solar plugged the board
stays powered and only goes dark when unplugged; the battery does **not** charge
while in ship mode. Correction to the PDF: ship mode wakes on the QON button as
well as on adapter insertion.

> TODO: transcribe the state diagram into text/mermaid here so it is searchable and
> reviewable in-repo, not only as a PDF.
