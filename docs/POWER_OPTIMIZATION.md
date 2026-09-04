# Power optimisation — work plan (bench session, week of 2026-09-08)

Everything below came out of the 2026-09-02 telemetry export
(`tools/tb_export_PCB-test-rig_20260902_1831.csv`, 137,939 datapoints,
2026-08-11 → 09-02). Nothing here needs a hardware change: every item is
firmware, on one side or the other.

Companion write-up on the module side: `Ultrasonic/Firmware/docs/DEVLOG.md` §13.

---

## 0. Before you start: the board's state

The unit has been indoors on battery since 08-17 with **no charging at all**
(deliberate max-lifespan test). At 09-02 it was at 3789 mV falling ~22.5 mV/day:

| threshold | what happens | ETA from 09-02 |
|---|---|---|
| 3600 mV | cellular stops, WiFi only | ~09-10 |
| 3450 mV | **all telemetry stops** — records bank in flash only | ~09-15 |
| 3350 mV | sampling drops to 30 min | ~09-19 |
| 3300 mV | park mode, 60-min wakes | ~09-20 |

So by the bench session it may already be silent on the dashboard. That is not
a fault. **Plug USB in first** — charging resumes, and the flash backlog drains
on the next session (ring holds 131k records, cursor is journalled, nothing is
lost). Grab that data before reflashing anything: a record-layout bump would
discard it.

---

## 1. What the run measured

Whole node: **114 mAh/day (~4.8 mA)** on the 5000 mAh cell — ~45 days
full-to-empty. Split (from ratios of voltage slopes, so independent of the
capacity calibration):

| load | share | cause |
|---|---|---|
| ultrasonic board | ~45% (~2.1 mA) | OPA836 idle ~1 mA **+** MSP430 never sleeping ~1 mA |
| ESP32 wakes | ~28% | 288 wakes/day |
| cellular | ~25% | measured: +3.08 mV per session, 2 sessions/day |
| ESP32 deep sleep | ~2% | — |

Reliability over the same window was excellent and is not a concern: 5812
records, ~286/day against 288 nominal, **zero crashes**, `uss_code` 122 on
100% of 5090 measurements, `snr_db` pinned at 34.0 dB for 18 days.

---

## 2. Work items

### A. Ultrasonic module firmware (G: drive repo)

- [ ] **A1. Sleep the main loop (biggest single item, ~1 mA).**
      `EcoTrace_Run()`'s `for(;;)` busy-polls forever — the MSP430 never leaves
      active mode (~120 µA/MHz from FRAM ≈ 1 mA at 8 MHz). Add
      `__bis_SR_register(LPM3_bits | GIE)` at the bottom of the loop for the
      `ECOTRACE_I2C_SLAVE` build.
      * the I²C ISR **already** exits LPM3 (`__bic_SR_register_on_exit`), and
        `uss_link.h` already documents this as the design — it was simply never
        implemented;
      * needs a timer wake for the autonomous tick (ACLK Timer_A or RTC);
      * `now_ms()` timebase must run off ACLK, not SMCLK, to survive LPM3;
      * watchdog interval must exceed the sleep period (or source it from
        ACLK) — `FEED_WDT()` stops happening while asleep;
      * keep the bench build busy-polling so UART stays responsive; the
        `ECOTRACE_I2C_SLAVE` split already exists for exactly this.

- [ ] **A2. PD-gate the OPA836 per capture (~1 mA).**
      In the autonomous tick: `board_rx_en(true)` → measure →
      `board_rx_en(false)`. **Leave the rails up** — the bias network and
      coupling caps stay charged, so there is no `RAIL_SETTLE_MS` and no
      discard capture, unlike full rail gating. Today `RX_EN` is only ever
      cleared as a side effect inside `board_pwr_rx(false)`, which the
      autonomous path reaches only at `AUTO_STOP`.
      *Acceptance:* `uss_amp_ups`/`uss_amp_dns`/`uss_snr_db` unchanged against
      the 34.0 dB / 18-day baseline. If they move, the amp needs more settle.

- [ ] **A3. (optional) Gate the boost + VCC_TX muxes between captures.**
      Tens of µA — the smallest prize, but the TPS61240 currently runs
      continuously to feed an idle level shifter. `board_boost()` already
      toggles it mid-capture for the "BOOST 2" noise mode, so it is proven.

- [ ] **A4. Fix the totalizer wrap.** At zero flow `uss_vol_ml` climbs ~17 L
      per 5 min and tops out at 4,294,437 mL. 2³²/1000 = 4,294,967: the
      accumulator is a **uint32 of µL reported as mL**, wrapping every ~21 h.
      Integrate signed, and either widen it or expose a wrap counter — the
      master reads only the absolute value every 5 min, so a wrap is
      indistinguishable from reverse flow.
      (The ~3.4 L/min phantom flow driving the wrap is the uncalibrated cell —
      known and accepted — but the wrap survives calibration.)

- [ ] **A5. Find out why `uss_tof_comp2_us` stopped on 2026-08-24.** 97 points
      that day, then nothing for 9 days while every other USS key kept flowing.
      Deliberate change or a code path that quietly stopped emitting?

- [ ] **A6. (optional) Latch min/max/mean flow since last read.** Makes a
      longer ESP sample interval lossless for diagnostics — 15-min records that
      still describe a 1 Hz process. Spare register space at 0x1C–0x1E.

- [ ] **A7. (optional, later) Adaptive rate** — slow when idle, fast on flow.
      Worth it for burst capture, *not* for power (see §3). Needs the zero-flow
      offset fixed first or the threshold sits permanently triggered.

### B. Communication module firmware (this repo)

- [ ] **B1. Restore `vac2_mv` and `ts_stat` (and ideally `tdie_c`) telemetry.**
      The 08-15 trim to 22 keys dropped `vac2_mv`, `vindpm_mv`, `vreg_mv`,
      `weather_good`, `eco_chg`, `tdie_c`, `ts_pct`, `ts_stat`, `press_mbar`.
      Consequence found this session: with no `vac2_mv` there is no way to tell
      "no panel voltage" from "panel fine, charger refusing". Indoors
      `light_ch0` settled it; outdoors that argument does not exist. Required
      before the panel swap.

- [ ] **B2. Log VBAT *during* modem transmit into the status record.** Every
      VBAT sample today is taken with the modem off, so the sag under the
      A7672's ~2 A burst has never been measured — and that sag, not energy, is
      what sets the minimum battery size. A single mid-session read in
      `cell_session()` turns battery sizing from a datasheet estimate into a
      measurement.

- [ ] **B3. Retune MPPT for the small panel** (do together with C1):
      `MPPT_VINDPM_START_MV` 14000 → ~5000, `MPPT_VINDPM_MIN_MV` 5000 → ~3800.
      The 5000 floor is our choice, not the chip's — the BQ25792 is
      **buck-boost**, VBUS range 3.6–24 V, so a 6 V panel at Vmp ~4.5 V when
      hot is still fine. Without the retune the MPPT clamps against the floor
      and sits off the maximum power point.

- [ ] **B4. Clear the NVS `voc_max` key when the panel changes.** It still
      holds the 18 V panel's best-ever Voc, and the poison guards deliberately
      make the reference slow to fall — the sun-hours detector would report
      permanent overcast. Needs a mechanism or a documented procedure.

- [ ] **B5. (optional, small) Trim per-wake cost.** LTR-303 costs 150 ms every
      wake for a reading that has been 0 for 18 days in the box — sample it
      every 12th wake. Outdoors, `mppt_step()` adds 600–900 ms per wake (3
      perturb steps at 150 ms plus an hourly 250 ms Voc); the operating point
      does not move on a 5-min timescale, so every Nth wake is enough. The
      200 ms console drain before sleep only matters on USB.

- [ ] **B6. (probably NOT needed) Sample interval 5 → 15 min, upload 12 → 24 h.**
      See §3 — after A1+A2 there is enough margin to keep 5-minute sampling.
      Change these for margin, not necessity.

- [ ] **B7. Decision to record: keep GPIO14 always-on.** Cutting the
      ultrasonic board's rail *is* firmware-only (the switch is fitted, and
      `I2C_LINK.md` §2.0 documents the SDA/SCL-low sequence needed to avoid
      back-feeding through the I²C ESD clamps). But after A1+A2 the module
      costs ~2.4 mAh/day, so the rail cut would save that last few percent and
      cost continuous totalizing. Not worth it.
      *Open question:* confirm the harness actually has the module on the
      switched rail rather than always-on VSYS.

### C. Hardware to have on the bench

- [ ] **C1. 6 V / 1 W panel** (~1 CHF) — works electrically; measure Voc in
      full sun before trusting the label. For the deployed fleet buy ETFE or
      proper laminate: cheap epoxy potting delaminates under UV within a year
      or two. Voltage is not the deciding factor — the buck-boost handles it.
- [ ] **C2. 18650, 2000–2500 mAh**, with protection PCB, solder tabs not a
      spring holder (holder contact resistance lands on the same sag budget).
      Sized by ESR for the 2 A modem burst, not by energy — 30–80 mΩ gives
      60–160 mV of sag. The 5000 mAh pack is ~2.5× larger than needed.

---

## 3. Expected outcome

| state | mAh/day | autonomy, 2500 mAh 18650 |
|---|---|---|
| today | 114 | 18 days |
| **after A1 + A2** | **~40** | **~50 days** |
| + B5/B6 if wanted | ~25 | ~80 days |

Against the 5–10 days of overcast that actually needs riding out, **A1 + A2
alone give 5× margin while keeping 5-minute sampling and 12-hour uploads.**
That is the headline: the fixes buy resolution back, not just runtime.

Derate ~25% for cell aging after two years at 45–57 °C enclosure temperature.
Recovery: a 1 W panel yields ~500–600 mAh on a decent day, so a drained pack
refills in 3–4 good days.

## 4. Verification

1. **Night slope** (00:00–05:00 UTC, no sun, no charging) is the clean metric —
   it isolates idle draw from cellular. Baseline to beat: **0.61–0.82 mV/h**.
   After A1+A2 expect roughly 0.15–0.25 mV/h.
2. **Signal integrity after A2:** `uss_amp_*` and `uss_snr_db` must be
   unchanged (34.0 dB baseline).
3. **Charger efficiency**, once B1 restores `vac2_mv`: plot
   `vbus_mv × ibus_ma` against `vbat_mv × ibat_ma` versus input power. This is
   what decides whether the BQ25792 is still the right chip at the new scale —
   its ~15 µA quiescent is negligible (~0.4 mAh/day), so the only real question
   is conversion efficiency at low irradiance. Decide with the curve, not on
   suspicion; a swap to a nano-power harvester (BQ25570, AEM10941) would cost
   the dual input, the full ADC set, JEITA and ship mode.
4. **Battery sag** from B2: headroom above the 3600 mV modem floor tells you
   how small a cell you can actually run.
