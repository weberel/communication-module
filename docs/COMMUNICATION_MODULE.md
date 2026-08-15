# ecoTrace Communication Module — system description

**Scope.** What this board is, how the firmware behaves in the field, what it
stores, and exactly what it sends to ThingsBoard. Written 2026-08-13 against
firmware **idf-0.10**. Companion document on the sensing side:
`Ultrasonic/Firmware/docs/I2C_LINK.md` (the link contract) — the two are
designed to be read together.

---

## 1. The plan

The module is the **communications and power half of a solar-powered remote
node**. It is the long-term replacement for a fleet of MicroPython
"bucket-communication" devices (20 units, six months in Kenya and Malawi):
same job, but engineered for unattended operation with no local support.

Design commitments, in priority order:

1. **Never lose data.** Sampling and logging must survive anything the uplink
   does. Records go to external flash first; the network is a separate,
   failable concern.
2. **Never expose the fleet.** All telemetry travels over TLS. There is no
   plaintext fallback at any escalation level — better no data than a leaked
   device credential or a compromised server.
3. **Never drain the battery chasing a network.** Outages are waited out, not
   fought. Roughly two short attempts per day; the log keeps banking meanwhile.
4. **Survive the season, not the demo.** Deep sleep between samples, a charge
   profile that respects battery chemistry and weather, and a park mode that
   refuses to write flash on a dying pack.

The node splits across two boards: **this one** (ESP32-C6: power, storage,
uplink) and the **ultrasonic flow module** (MSP430FR6043 + gas cell). This
board owns the wake schedule, so it owns the I²C bus; the ultrasonic module
joins as a slave peripheral. See §7.

---

## 2. The hardware (Rev A, 2026-04-23)

Single ESP32-C6-MINI-1 board. Pin map source of truth:
[`lib/EcoTrace/ecotrace_pins.h`](../lib/EcoTrace/ecotrace_pins.h) — verified
against the schematic 2026-05-05.

### 2.1 Blocks

| Block | Part | Interface | Notes |
|---|---|---|---|
| MCU | ESP32-C6-MINI-1 | — | 4 MB flash used; RISC-V + LP core (LP core unused so far) |
| Charger / PMIC | BQ25792 | I²C 0x6B | 1S Li-ion, dual input (USB + solar), MPPT via VINDPM, full ADC set, JEITA |
| Cellular | SIMCom A7672E-LASE | UART1 @115200 | LTE Cat-1, PPP; separate switched rail |
| External storage | GD25Q128 (16 MB NOR) | SPI | the log ring; deep-power-down between wakes |
| Light | LTR-303 | I²C 0x29 | always-on 3V3 rail |
| Accelerometer | SC7A20 | I²C 0x18/0x19 | tilt/tamper; always-on rail |
| Barometer | MS5837-02BA | I²C 0x76 | pressure + temperature |
| Secure element | ATECC608B | I²C 0x60 | fitted, **not yet used** (mTLS endgame) |
| Flow module | MSP430FR6043 (separate PCB) | I²C 0x2C | slave; see §7 |
| Pressure (flow board) | WF280A | I²C 0x38 | read directly by this board; see §7 |

### 2.2 Pins that matter

```
I²C  SDA 6, SCL 7 (100 kHz, external 4k7 pull-ups)
SPI  MOSI 4, MISO 5, CLK 18, CS_flash 8
Modem TX 20, RX 21, PWRKEY 22, PWR_EN 23, STATUS 0
Wake  QON/button 2 (EXT1), sensor INT 3, BQ INT 19
Misc  LED 1, external sensor rail 14 (active HIGH)
```

Two hardware behaviours the firmware must respect:

- **PWRKEY is inverted** by Q409: GPIO HIGH = PWRKEY asserted at the modem.
  It is held **LOW through deep sleep** — holding it asserted burns ~0.3 mA in
  the inverter (lesson from dl-2.10).
- **The on-board sensors are on the always-on 3V3 rail**, not the switched
  GPIO14 rail. They are low-powered over I²C, never by cutting their supply.

### 2.3 Power tree

Solar panel (1.2 W) and/or USB → BQ25792 → 1S LiPo (2500 mAh) → RT9080 LDO →
3V3 always-on rail (ESP32 + sensors + flash). The modem has its own switched
high-side rail off VBAT, enabled only for an uplink session. GPIO14 switches an
external-sensor header rail, independent of the on-board sensors.

---

## 3. How the firmware works

Source: [`idf/src/`](../idf/src). ESP-IDF (PlatformIO `framework = espidf`,
pioarduino platform 53.03.13). The Arduino implementation in `src/datalogger`
is **frozen at dl-2.14** as a hardware-validated reference — do not modify it.

### 3.1 The wake cycle

Every wake is a complete, self-contained transaction — there is no long-running
main loop. `app_main()` runs once and ends in deep sleep.

```
wake (timer / button / crash-reboot)
  ├─ enforce 120 s task watchdog          (runtime, not just sdkconfig — §3.6)
  ├─ board_init(): rails safe, PWRKEY idle
  ├─ BQ25792 up, read VBAT
  ├─ park-mode check ──────────────► if parked: sleep 3600 s, done
  ├─ flash log begin (recover cursors if cold/crashed)
  ├─ solar_on_wake(): MPPT step, harvest accounting, weather verdict, VREG target
  ├─ read_sample(): charger + all sensors + flow module → one 128 B record
  ├─ append record to the flash ring
  ├─ upload if due (12 h schedule, or button) ─► §4
  └─ deep sleep (300 s; ×6 when critically low)
```

State that must survive sleep (ring cursors, MPPT operating point, harvest
counters, upload schedule, boot/crash counters) lives in **RTC RAM**, with the
flash-log cursors additionally journalled to flash sector 0 so a power cut is
recoverable. A cold boot rebuilds the ring cursors by forward-scanning the
chip.

### 3.2 Sampling and the flash ring

- Interval **300 s** (5 min) during validation. For deployment this is expected
  to drop to 10–15 min, ideally by remote configuration.
- One **128-byte record** per sample, self-validating: magic + sequence number
  + CRC16-CCITT. Fixed size keeps the ring trivial — 32 records per 4 KB
  sector, never crossing a page.
- Capacity: 4095 data sectors × 32 = **131,040 records ≈ 1.25 years** at 5 min.
  Records are only dropped from the tail when the ring wraps, and the upload
  cursor only advances on a broker **PUBACK**, so an unacknowledged record is
  re-sent rather than lost.
- Record magic is a layout version: `ECL3` → `ECL4` at idf-0.10 (64 → 128 B,
  flow-module fields added). **A magic bump makes older pending records
  unreadable**, so a build carrying one must be flashed immediately after a
  clean upload.

### 3.3 Charging, MPPT and weather

- **MPPT** on the BQ25792's VINDPM input-regulation point: fractional-Voc
  (80 % of a periodically measured open-circuit panel voltage) plus a small
  perturb-and-observe hill climb, 3 steps per wake, clamped 5–22 V.
- **Weather-aware charge target.** A "good day" caps the pack at 4050 mV
  (~80 % SoC) instead of 4200 mV — meaningfully longer calendar life. USB
  always charges full: plugging a cable in is a deliberate "fill it up".
- Two independent signals declare a good day; either suffices:
  - **harvest** — integrated input current ≥ 1000 mAh today or yesterday;
  - **Voc sun-hours** — hourly open-circuit panel voltage ≥ 80 % of a
    self-calibrating best-ever Voc (persisted in NVS, with guards against one
    glitched reading poisoning the reference), ≥ 4 such hours.

  The second exists because harvest goes blind when the battery is already
  full — found in the 2026-08-12 telemetry audit.
- **JEITA** thermal charge suspension is the charger's own doing and is now
  fully observable in telemetry (`tdie_c`, `ts_pct`, `ts_stat`) after an
  afternoon lockout at ~57 °C box temperature was misread as a fault.

### 3.4 Battery protection

| Threshold | Value | Behaviour |
|---|---|---|
| Cellular floor | 3600 mV | no modem attempts below (resting voltage) |
| WiFi floor | 3450 mV | no WiFi attempts below |
| Park entry | 3300 mV | no flash writes, no sensors, 3600 s sleeps |
| Park exit | 3450 mV | hysteresis |
| Critical sampling | 3350 mV | sample interval ×6 (30 min) |

Park mode exists to avoid the one unrecoverable failure: a brownout in the
middle of a flash write.

### 3.5 Clock

Correct timestamps matter more than they look — every record carries one, and a
wrong clock silently corrupts a year of data. Carrier NITZ is **never** used
(it produced a 1970→2070 parse bug). Instead: multi-server SNTP
(time.google.com, pool.ntp.org) cross-checked against the **HTTPS `Date`
header** from the ThingsBoard server. The carrier can intercept UDP NTP; it
cannot rewrite a header inside our TLS session, so a disagreement > 120 s
vetoes the SNTP result. Sanity bounds: [2026-01-01, 2036-01-01). Re-synced
every uplink session. A record sampled before the first sync stores `ts_s = 0`
and is timestamped at upload from its position in the ring.

### 3.6 Watchdog

120 s task watchdog, panic on expiry. It is set **at runtime** with
`esp_task_wdt_reconfigure()` and not only in `sdkconfig.defaults`: regenerating
the sdkconfig silently reverted the timeout to 5 s once and boot-looped the
board during the modem's 8 s settle (idf-0.5/0.6).

---

## 4. Uplink: what goes to ThingsBoard, and how

### 4.1 Transport

**Cellular is primary, WiFi is backup** — the reverse of what is convenient on
a bench, but correct for the field.

```
cell_session:  modem rail on → PWRKEY → 8 s fed settle → esp_modem sync
               → +CEREG registration (stat 1 or 5) → PPP up
               → clock sync → MQTT → drain → status record
wifi_session:  same publisher, different netif
```

Registration is bounded: 25 s fail-fast if CSQ still reports no signal, 45 s
normal attach budget, 180 s once-daily deep search.

**Broker:** `mqtts://eu.thingsboard.cloud:8883`, TLS with the **ISRG Root X1**
certificate pinned into the image. Authentication is the ThingsBoard device
access token as the MQTT username. **There is no plaintext fallback** — if TLS
cannot be established the session fails and the data waits in flash.

**Topic:** `v1/devices/me/telemetry`, QoS 1.

### 4.2 Batching and pacing

Records are drained oldest-first in batches of **8**, as a JSON array, with a
400 ms gap between batches (ThingsBoard Cloud dislikes bursts). The ring cursor
advances only on PUBACK. Broker connection churn — routinely ~18 s on TB Cloud
— is handled by waiting out the client's auto-reconnect (up to 15 s) and
retrying the publish once.

A **status record** is published at both the start and the end of every
session. Sending it only at the end lost it nearly every time to the
end-of-session disconnect; health telemetry must not depend on the drain
succeeding.

### 4.3 Payload format

Historical records carry an explicit timestamp:

```json
[{"ts": 1786000000000, "values": {"vbat_mv": 3987, "ibat_ma": 142, ...}},
 {"ts": 1786000300000, "values": {...}}]
```

Status records are a single object of the same shape.

### 4.4 Telemetry keys

**Per record** (every sample):

| key | unit | meaning |
|---|---|---|
| `vbat_mv` | mV | battery voltage |
| `ibat_ma` | mA | battery current (+ charging, − discharging) |
| `bat_mw` | mW | derived battery power |
| `soc_pct` | % | crude voltage-based state of charge (temperature-sensitive — §6) |
| `vbus_mv`, `ibus_ma` | mV, mA | charger input rail |
| `vac2_mv` | mV | solar input voltage |
| `vsys_mv` | mV | system rail |
| `vindpm_mv` | mV | MPPT operating point |
| `vreg_mv` | mV | active charge-voltage target (4200 or 4050) |
| `chg_stat` | enum | BQ25792 charge state |
| `fault0`, `fault1` | bitfield | BQ25792 fault registers |
| `harvest_mah` | mAh | solar charge harvested today |
| `solar`, `usb` | 0/1 | input present |
| `weather_good`, `eco_chg` | 0/1 | weather verdict, eco charge cap active |
| `light_ch0`, `light_ch1` | counts | LTR-303 visible+IR, IR |
| `acc_x_mg`, `acc_y_mg`, `acc_z_mg` | mg | SC7A20 |
| `press_mbar`, `temp_c` | mbar, °C | MS5837 |
| `tdie_c` | °C | BQ25792 die temperature |
| `ts_pct` | % | battery NTC as % of bias |
| `ts_stat` | bitfield | charger JEITA verdict |
| `sensor_ok` | bitfield | bit0 BQ, bit1 LTR-303, bit2 SC7A20, bit3 MS5837, bit4 flow module, bit5 WF280A |

`sensor_ok` matters: without it a zero reading is indistinguishable from a dead
sensor.

**Per record, only when the flow module answered** (`sensor_ok` bit 4):
`flow_lpm` (L/min), `uss_dtof_ns` (ns), `uss_temp_c` (°C), `uss_code`
(122 = valid, 126 = no echo), `uss_amp_ups`, `uss_amp_dns` (ADC counts),
`uss_snr_db` (dB), `uss_gain` (index), `uss_vol_ml` (mL, totalizer — reserved),
`uss_status` (link status bits).

Signal metrics are sent even when no echo was found; that is what makes a
remote gain problem diagnosable from a dashboard.

**Per record, only when the WF280A answered** (bit 5): `wf_praw`, `wf_traw` —
raw 24-bit counts. The vendor compensation polynomial is private, so
conversion to physical units happens server-side once characterized.

**Per session** (status record): `fw` (version string), `transport`
(`cell`/`wifi`/`none`), `rssi_dbm`, `wifi_rssi_dbm`, `cereg_stat`, `backlog`
(records still pending), `boot_id`, `boot_count`, `wake_count`, `crash_count`,
`reset_reason`, `vbat_mv`, `sun_h`, `voc_max_mv`.

### 4.5 Schedule and escalation

Uploads every **12 h**, plus a manual button press. On failure: up to **2
retries at 30 min**, then back to the 12 h schedule. Escalation varies
*technique*, never *tempo* — a multi-day outage costs roughly two short
attempts per day while everything banks in flash. If an upload attempt crashes
the board, the next attempt skips cellular once (one-shot) and tries WiFi.

**Datapoint budget:** ~26 keys × 288 samples/day ≈ 7.5 k datapoints/device/day
at the current 5-minute interval, more with the flow keys. Fine for validation;
for a fleet, lengthen the interval and/or send daily aggregates.

---

## 5. Build, flash, deploy

```bash
cd idf && pio run                 # build
pio run -t upload                 # normal flash
pio device monitor                # 115200
```

Partition layout (4 MB): dual 1.9 MB OTA slots with bootloader rollback
enabled, 64 KB coredump, NVS.

**ESP32-C6 flashing quirk that will bite anyone new:** the USB-Serial/JTAG port
only exists while the chip is awake, and *opening the port resets the chip*
(reset reason 11), wiping RTC RAM. On a deep-sleeping unit, flashing means
catching a wake window — a background watcher that waits for the port to
appear, then writes the app only at `0x10000`. Observe a running unit
sparingly: every serial connection is a cold boot.

Secrets (`idf/src/secrets.h`, gitignored): APN, broker URI, device access
token, WiFi credentials.

---

## 6. Field lessons worth carrying forward

- **Afternoon "battery drain" that isn't.** A hot pack shows depressed
  open-circuit voltage (~50 mV at +30 °C) which recovers on cooling. It looks
  like drain in `vbat_mv`/`soc_pct` with `ibat_ma` at zero. Real, reversible,
  and a reason not to trust voltage-based SoC in the heat.
- **Enclosure temperature drives everything.** At ~57 °C box temperature the
  charger's JEITA protection suspends charging 12:00–18:00 UTC. The hardware is
  behaving correctly; the enclosure is the problem.
- **The light sensor is nearly useless inside a box.** Panel Voc is the better
  weather signal — it exists whether or not charging is happening.
- **Broker churn is normal**, not a fault. Publishes must be retry-wrapped.

## 7. Interface to the ultrasonic flow module

One bus, one master: **this board is the master**, because it owns the wake
schedule. The MSP430FR6043 is an I²C **slave at 0x2C**, and this board reads
the flow board's WF280A pressure sensor (0x38) directly as another device on
the same wires.

Handshake per sample: write `MEASURE` to the command register, poll status
until ready (3 s budget), read a 28-byte CRC8-guarded result block in one
transaction. Integers only on the wire — flow in µL/min, ΔToF in picoseconds,
temperature in 0.01 °C, SNR in half-dB.

The contract is `uss_link.h`, held **byte-identical in both repositories**
(`idf/src/uss_link.h` and `Ultrasonic/Firmware/fw/uss_link.h`); they must be
changed together. Master-side driver: `idf/src/uss.c`. Full rationale, wiring,
power constraints and bring-up checklist:
**`Ultrasonic/Firmware/docs/I2C_LINK.md`**.

Two constraints that belong here as well as there:

- The flow board's `/VCC` must be fed from this board's always-on rail. An
  unpowered board on a shared bus drags SDA/SCL down and takes out every other
  sensor.
- The flow module's own I²C master (its bit-banged WF280A driver) must be
  dormant in the deployed build. Two masters on one bus is a bus fight.

**Status (2026-08-15): WORKING on hardware, end to end.**

Verified: `sensor_ok=0x3F` (all six devices), `uss_code=122`, absolute ToF
131.3 us -> 335 m/s, autonomous mode `st=0x09`, volume accumulating, records
uploading over LTE with the backlog draining to zero.

### How it is driven

The module runs **autonomously at 1 Hz** and integrates flow into `VOL_ML`; this
board wakes every 5 min and reads the latest latched block plus the totalizer.
Sample rate is decoupled from bus traffic, which is also the prerequisite for
LP-core sampling later. Rules the master must follow are in
`Ultrasonic/Firmware/docs/PROTOCOL.md` section 8.3 -- in particular: only send
`AUTO_START` when `STATUS.AUTO` is clear (it zeroes the totalizer), wait for
`AUTO` and then for `READY` before sampling, and never send `MEASURE` while AUTO
is set.

### Power, and why it matters more than it looks

The ultrasonic board is on the **switched SENSOR rail (J404, GPIO14)**, and the
rail is **always on** except during a deliberate power cycle. Three findings
force that:

1. **An unpowered board on the shared bus clamps it.** Its ESD diodes hold
   SDA/SCL near 0.6 V and its own 4k7 pull-ups become pull-downs to a dead rail.
   The symptom is every device unreachable -- the BQ25792 on our own always-on
   rail included -- and a record of all zeros.
2. **Leaving the rail off in sleep costs power, not saves it**: our pull-ups
   then feed the sleeping board through those clamps, ~1.4 mA continuously.
3. **Cutting VCC does not power it down while the bus idles high.** To
   power-cycle it you must hold SDA/SCL LOW for the whole off window --
   `board_sensor_power_cycle()` does this, and `eco_i2c_hold_low()` tears the
   bus down to make it possible. **CAUTION: that blocks the charger and every
   other sensor for the duration, so keep the window short.**

`BOARD_SENSOR_BOOT_MS` is **3000**: that board's init primes the USSXT crystal
with up to 5 x 300 ms retries, and talking to it sooner gets a clean NACK from a
module that is merely still booting.

### Telemetry keys

22 per record (trimmed from ~40 on 2026-08-15; the 128-byte flash record still
carries every field, only the uplink is trimmed):

`vbat_mv ibat_ma soc_pct vbus_mv ibus_ma chg_stat harvest_mah solar usb`
`light_ch0 acc_x_mg acc_y_mg acc_z_mg temp_c sensor_ok`
`p_gas_hpa dp_hpa`
`uss_tof_us uss_dtof_us uss_code uss_snr_db uss_vol_ml`

* `uss_tof_us` is the **mean** of both directions: the mean is the
  speed-of-sound (composition) signal, the difference is flow and `uss_dtof_us`
  carries that at far better resolution.
* `uss_vol_ml` is a counter to be **differenced**; it wraps at 4294 L.
* `flow_lpm` is deliberately **not** published: the VFR constants are wrong for
  this cell, so it is misleading rather than merely useless.
* Datapoint budget matters -- at ~40 keys x 8 records ThingsBoard Cloud closed
  the MQTT connection.

## 8. Roadmap

In rough priority order:

1. **OTA with rollback** — staged through the 16 MB external flash with a
   golden image. This is what ends bench visits; the partition table and
   bootloader rollback are already in place.
2. **Remote configuration** via ThingsBoard shared attributes — per-device
   charge target (server-side weather-informed, no on-device forecast API),
   sample interval, "stay awake for OTA". NVS-cached with a freshness window,
   falling back to the on-board Voc heuristic.
3. **Error-event log** in the coredump partition; health telemetry.
4. **NVS provisioning** — one image for the whole fleet.
5. **ATECC608B identity**, ending in mTLS. TLS must terminate on the ESP32, not
   in the modem's AT stack, for the secure element to be usable.
6. **LP-core sampling** while the HP core sleeps.
7. **Autonomous flow totalizer** on the MSP430 (register space reserved): flow
   integrated between samples is a far better volume estimate than 288
   instantaneous readings, and costs this board nothing.
