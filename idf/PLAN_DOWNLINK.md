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

## Stage 1: config (one build, com-0.35) -- CODED 2026-09-30, not yet run on hardware

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

Done when (hardware, still open): a config published from the server shows up in the next
status message with the same `cfg_ver`; the following wakes are spaced by the new interval
(USB console timestamps); a rejected payload shows `cfg_err` and unchanged intervals; a board
that never received a config reports `cfg_ver` 0 and the compiled 300 / 43200; a session
before the ACL is deployed logs "config subscribe refused" and drains normally.

## Stage 2: OTA (com-0.36)

Files: `uplink.c`, `devcfg.c`, `main.c`; `esp_https_ota` added to `PRIV_REQUIRES`.

1. After the drain (data first), if `fw_ver` is present, differs from `FW_VERSION`, and is not
   the version recorded in NVS as `fw_failed` with 2 attempts: download with `esp_https_ota`
   over the link already up (PPP or WiFi), same pinned ISRG root as MQTT. Check the SHA-256 of
   the written image against `fw_sha256` before `esp_ota_set_boot_partition`. 1.3 MB at
   230400 baud is about a minute of modem time, once.
2. Before `esp_restart()`: clear the RTC magic. The new image has a different `RTC_DATA_ATTR`
   layout and must come up as a cold boot, which also makes it phone home immediately.
   Without this the first wake counts as a crash and skips cellular.
3. Rollback. `esp_ota_mark_app_valid_cancel_rollback()` moves from the top of `app_main` to
   the end of the first upload session, called only if that session published. Deep-sleep wake
   goes through the bootloader, so an image that reaches deep sleep unmarked is rolled back by
   the bootloader on the next wake: a build whose uplink is broken reverts itself within one
   sample interval and the server sees the old `fw` again. An image that panics before that
   point is rolled back as today. Rule from the 2026-09-11 incident stays: never flash over USB
   into a slot without checking which one boots.
4. The rolled-back image finds the same retained config, so it records `fw_ver` in NVS with an
   attempt counter and stops after 2, reporting `fw_err: rolled_back`. The server clears it by
   publishing a new `fw_ver`.
5. The USB and WiFi-button OTA paths stay as they are.

Done when: a version published from the server installs over cellular, the next status message
shows the new `fw` and the old `cfg_ver`; a deliberately broken image (uplink disabled) reverts
within two wakes and reports `fw_err`; a wrong SHA-256 is refused before the slot is switched.

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
