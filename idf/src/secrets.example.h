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
