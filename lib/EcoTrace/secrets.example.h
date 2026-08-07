/*
 * secrets.example.h -- copy this file to secrets.h and fill in your own values.
 * secrets.h is gitignored so real credentials never get committed.
 *
 *     cp lib/EcoTrace/secrets.example.h lib/EcoTrace/secrets.h
 */
#pragma once

/* Cellular: your SIM operator's APN. */
#define SIM_APN        "internet"

/* Telemetry endpoint: a ThingsBoard device HTTP endpoint. Used by the datalogger
 * for both cellular and WiFi uploads (and by the functionality test's POST check).
 * Leave undefined for bench mode: the datalogger samples and prints over serial
 * but does not transmit. */
// #define POST_URL    "http://demo.thingsboard.io/api/v1/YOUR_TOKEN/telemetry"

/* WiFi backup uplink: used by the datalogger when cellular fails, and by the
 * button-triggered WiFi OTA mode. Leave undefined to disable both. */
// #define WIFI_SSID   "your-ssid"
// #define WIFI_PASS   "your-password"

/* Optional: password for WiFi OTA pushes (ArduinoOTA). If set, add
 * `upload_flags = --auth=<password>` to the datalogger_ota env. */
// #define OTA_PASSWORD "change-me"
