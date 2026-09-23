/*
 * secrets.example.h -- copy to secrets.h (gitignored) and fill in.
 * Interim: compiled-in like the Arduino build. The provisioning milestone moves
 * all of this into NVS so one image serves the whole fleet.
 */
#pragma once

/* Cellular */
#define SIM_APN         "internet"

/* ThingsBoard over MQTTS (TLS only -- there is no plaintext path to enable). */
#define TB_MQTT_URI     "mqtts://eu.thingsboard.cloud:8883"
#define TB_ACCESS_TOKEN "YOUR_DEVICE_TOKEN"

/* WiFi backup uplink; leave undefined to disable the WiFi path. */
// #define WIFI_SSID   "your-ssid"
// #define WIFI_PASS   "your-password"

/* ---- ecoTrace server (replaces ThingsBoard) --------------------------------
 * Credentials are per device and supplied out of band -- never in git.
 * The broker ACL is `pattern write ecotrace/%u/telemetry`, so MQTT_TOPIC MUST
 * carry the same name as MQTT_USERNAME or every publish is refused. */
#define MQTT_URI       "mqtts://ingest.ecotrace.ch:8883"
#define MQTT_USERNAME  "dev-N"                    /* <-- FILL IN */
#define MQTT_PASSWORD  "0000000000000000000000000000000000000000"  /* <-- FILL IN */
#define MQTT_TOPIC     "ecotrace/dev-N/telemetry" /* <-- FILL IN, must match username */
