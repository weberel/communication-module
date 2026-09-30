# Plan: downlink (remote config and OTA) for the comm board

Written 2026-09-30, before the code. **The contract both sides build against is
`ecotrace-server/docs/DOWNLINK.md`** (agreed the same day, with the three corrections below
taken in). This file is the firmware half: stages, files, done-criteria. Where the two differ,
DOWNLINK.md wins.

## Contract

**Topic.** `ecotrace/<username>/config`, retained, QoS 1, published by the server only. The
device gets `pattern read ecotrace/%u/config` in the broker ACL and nothing else new. It is not
the telemetry topic: a retained message there would be ingested as a reading, and the device
must never be able to write its own config.

**Payload.** One JSON object. Unknown keys are ignored, so the contract can grow.

```json
{"cfg_ver": 7,
 "sample_interval_s": 300, "upload_period_s": 43200,
 "fw_ver": "com-0.36", "fw_url": "https://ingest.ecotrace.ch/fw/com-0.36.bin",
 "fw_sha256": "<64 hex>"}
```

- `cfg_ver` is an identity, not an order: the device applies a payload whose version differs
  from the one in NVS. A server that resets the counter still gets applied.
- Interval bounds on the device: sample 60..3600 s, upload 600..86400 s, upload >= sample.
  A payload that fails them is not applied at all and is reported with `cfg_err: "range"`.
- The `fw_*` keys are optional. Present means "run this version"; absent means keep what runs.
- An empty retained payload (the server's "clear") means no config: the device keeps its NVS
  values and reports them. Not an error.

**Ack, in the status message.** `cfg_ver` (in effect, 0 if never configured), `sample_s`,
`upload_s` (in force for the sleep that follows), `cfg_err` (only while the last received
payload was rejected: `parse`, `range`), and after a failed OTA `fw_err` (`download`, `sha256`,
`write`, `rollback`) with `fw_err_ver`. `fw` is already sent on every status message and is the
OTA ack.

The ack is truthful within one session: the payload arrives at subscribe time, before the
drain; the sleep length and the upload countdown are computed after the drain, so the status
message of the same session reports what will actually happen next.

## Stage 1: config (one build, com-0.35) -- DONE, verified on unit A (dev-4) 2026-09-30

Files: `main.c`, `uplink.c`, new `devcfg.c/h`, `config.h`, `CMakeLists.txt` (adds `json`).

- [x] `devcfg.c`: NVS namespace `cfg`, keys `ver`, `samp_s`, `upl_s` (u32). `devcfg_init()`
      at every boot in `app_main`, right after the rollback mark and before `board_init()`; it
      owns `nvs_flash_init()` now (it used to run only on the WiFi path). Values cached in
      `RTC_DATA_ATTR` with a magic: a warm wake never opens NVS, a cold boot reloads.
- [x] `SAMPLE_INTERVAL_S` / `UPLOAD_PERIOD_S` are compiled defaults only (`config.h`); the five
      use sites read `devcfg_sample_s()` / `devcfg_upload_s()`.
- [x] `uplink.c`: `config_fetch()` after `mqtt_up()` in both the cellular and the WiFi session,
      before `drain()`. Subscribes QoS 1 to `ecotrace/<user>/config`, waits up to 2 s for
      `EV_CFG_DONE`, unsubscribes. A refused SUBSCRIBE (`MQTT_ERROR_TYPE_SUBSCRIBE_FAILED`,
      the ACL not deployed yet) and a timeout are both "no config". The payload is applied
      from the MQTT event task (`devcfg_apply`, cJSON, bounded parse).
- [x] Apply rule as in DOWNLINK.md: `cfg_ver` differs from NVS, absent interval = keep, bounds
      sample 60..3600, upload 600..86400, upload >= sample checked on the resulting pair; any
      violation rejects the whole payload. NVS written and committed before the cache moves.
- [x] Status keys: `cfg_ver`, `sample_s`, `upload_s` on every status message, `cfg_err`
      (`parse` | `range`) only while the last received payload was rejected.
- [x] No countdown clamp needed: the countdown is (re)armed from `devcfg_upload_s()` after
      the session, so a new period is in force from the same session.

Verified 2026-09-30 from the server UI, board untouched: a config published from the server shows up in the next
status message with the same `cfg_ver`; the following wakes are spaced by the new interval
(USB console timestamps); a rejected payload shows `cfg_err` and unchanged intervals; a board
that never received a config reports `cfg_ver` 0 and the compiled 300 / 43200; a session
before the ACL is deployed logs "config subscribe refused" and drains normally.

## Stage 2: OTA (com-0.36) -- DONE: forward and reverse OTA proven on dev-4 2026-09-30; rollback with a broken build still open

Files: `ota.c/h` (new), `devcfg.c/h`, `uplink.c/h`, `main.c`, `config.h`, `src/CMakeLists.txt`,
`version.txt` (new).

- [x] **One version source.** `idf/version.txt` holds `com-0.36`; ESP-IDF puts it in the image's
      app descriptor (the server reads it there at release upload) and `src/CMakeLists.txt`
      passes the same string as `FW_VERSION`. `config.h` errors if it is missing. Bump
      `version.txt` **and touch `idf/CMakeLists.txt`** (PlatformIO re-runs CMake only when a
      CMakeLists changes; a bump alone rebuilt com-0.36 on 2026-09-30). Checked on the built
      image: descriptor and status string agree. Release copies go to `.pio/build/release/`.
- [x] `devcfg.c` takes `fw_ver` / `fw_url` / `fw_sha256` from every accepted payload (also on
      `same`), all three or none; keeps them for the session. Attempt counter per version and
      the last error (`fw_err`, `fw_err_ver`) in NVS, so they outlive a rollback.
- [x] `ota.c`: `esp_https_ota` begin/perform/finish over the link already up, pinned ISRG root,
      into `esp_ota_get_next_update_partition()`. SHA-256 of the written bytes (mbedtls, 4 KB
      static buffer, WDT reset per chunk) against `fw_sha256` BEFORE `esp_https_ota_finish()`
      switches the boot partition. Errors map to the contract: `download`, `sha256`, `write`.
- [x] `uplink.c` `ota_attempt()`: after the drain, before the status message, in both the
      cellular and the WiFi session. Skipped when the target equals the running version, when
      VBAT < modem floor + 100 mV, or after 3 attempts for that version. The attempt is counted
      before the download starts, so a crash mid-download counts.
- [x] `main.c`: the mark-valid call moved from the top of `app_main` to after the upload
      session, only when it published. `ota_boot_check()` after the boot classification: an
      ABORTED other slot records `fw_err: rollback` with that slot's descriptor version and
      forces an upload now. On `ota_ready`: RTC magic cleared, devcfg cache dropped, restart.
- [x] Status keys `fw_err`, `fw_err_ver` when set; `fw` unchanged (it is the ack).
- [x] Builds clean: RAM 19.5 %, flash 64.8 %.
- [x] **Found on the first attempt, fixed the same day:** the download wedged after
      `uart_terminal: HW FIFO Overflow`. The UART ISR was in flash, so it is masked during every
      OTA flash write and modem bytes are lost, which killed the PPP stream; and
      `esp_https_ota_perform()` returns IN_PROGRESS on a read timeout, so the loop spun for 10+
      minutes feeding the watchdog with the modem on. Fixes: `CONFIG_UART_ISR_IN_IRAM=y`
      (sdkconfig.defaults and the generated sdkconfig), a 60 s no-progress stall and a 9 min
      deadline in `ota_run()` (`fw_err: download`), and the attempt counter restarts when the
      running version changes (a serial reflash gets fresh tries).
- [x] Forward OTA verified 2026-09-30 17:06: dev-4 on the fixed com-0.36 downloaded com-0.37
      over cellular, restarted, reported `fw: com-0.37`, cfg_ver 4 applied, server "up to date".
      (That com-0.37 was the build WITHOUT the UART fix; it must not be asked to OTA again.)

- [x] With the fixed code on both ends, 2026-09-30 17:19 and 17:26: com-0.36 -> com-0.37 -> com-0.36
      over cellular, each under 4 min from button press to "up to date" on the server. The status
      message of the downloading session goes out with the old `fw` and no `fw_err`, then the
      new image's cold-boot session acknowledges.

Still open on hardware: an image built with the uplink disabled reverts within two
wakes and the old image reports `fw_err: rollback` with `fw_err_ver`; a wrong SHA-256 is
refused with `fw_err: sha256` and the running image is untouched.

## Server side: three corrections to the 2026-09-30 plan (all taken into DOWNLINK.md)

1. Separate topic and a read ACL entry, see Contract. "The existing topic" does not work.
2. Put the `fw_*` keys and the error keys into the contract now, so the handoff is written
   once. The server leaves them unset until it hosts images.
3. Host firmware images over HTTPS behind the existing certificate chain (Caddy, same host as
   ingest), with the SHA-256 computed at upload time and stored with the version.

Per device only, no group inheritance. Agreed. Deltas the server side added: `fw_err` values
are `download | sha256 | write | rollback` with `fw_err_ver`; the OTA attempt counter stops at 3;
`cmd` stays reserved for one-shot commands.

## Traps
- `nvs_flash_init()` currently runs only when WiFi is used (`uplink.c` 1078). It must run at boot.
- ESP-IDF rollback is checked in the bootloader on every reset, including deep-sleep wake.
- MQTT 5 stays: a refused subscribe is visible as a reason code, a 3.1.1 broker would hide it.
- `boot_id` rolls on the post-OTA cold boot by design; the server's gap check treats the two
  boots separately.
