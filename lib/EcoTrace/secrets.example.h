/*
 * secrets.example.h -- copy this file to secrets.h and fill in your own values.
 * secrets.h is gitignored so real credentials never get committed.
 *
 *     cp lib/EcoTrace/secrets.example.h lib/EcoTrace/secrets.h
 */
#pragma once

/* Cellular: your SIM operator's APN. */
#define SIM_APN        "internet"

/* Optional: an HTTP endpoint to POST to (e.g. a ThingsBoard telemetry URL).
 * Leave undefined to skip the POST test in 04_cellular_http. */
// #define POST_URL    "http://demo.thingsboard.io/api/v1/YOUR_TOKEN/telemetry"

/* Optional: WiFi credentials, if you add a WiFi transport example. */
// #define WIFI_SSID   "your-ssid"
// #define WIFI_PASS   "your-password"
