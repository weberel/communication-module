#include "ota.h"
#include "config.h"
#include "EcoTraceBoard.h"
#include "esp_task_wdt.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#if defined(WIFI_SSID) && defined(WIFI_PASS)
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

namespace Ota {

#if defined(WIFI_SSID) && defined(WIFI_PASS)

static volatile bool s_updating = false;

void runWindow()
{
    Serial.printf("OTA mode: joining '%s'...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(250);
        EcoTrace::ledToggle();
        esp_task_wdt_reset();
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("OTA mode: wifi failed -- resuming normal cycle");
        WiFi.mode(WIFI_OFF);
        EcoTrace::ledOff();
        return;
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() {
        s_updating = true;
        EcoTrace::ledOn();
        Serial.println("OTA: receiving image...");
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        esp_task_wdt_reset();
        static uint8_t last_pct = 255;
        uint8_t pct = (uint8_t)(100U * done / total);
        if (pct / 20 != last_pct / 20) { Serial.printf("  %u%%\n", pct); last_pct = pct; }
    });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: done, rebooting into new image"); });
    ArduinoOTA.onError([](ota_error_t e) {
        s_updating = false;
        EcoTrace::ledOff();
        Serial.printf("OTA: error %d\n", (int)e);
    });
    ArduinoOTA.begin();

    Serial.printf("OTA mode: listening as %s.local (%s) for %d s\n"
                  "  push with: pio run -e datalogger_ota -t upload\n",
                  OTA_HOSTNAME, WiFi.localIP().toString().c_str(), OTA_WINDOW_S);

    /* Wait out the window (or however long an in-flight update needs). A short
     * LED blip every second says "I'm in OTA mode". */
    uint32_t deadline = millis() + (uint32_t)OTA_WINDOW_S * 1000;
    uint32_t next_blink = 0;
    while ((int32_t)(deadline - millis()) > 0 || s_updating) {
        ArduinoOTA.handle();
        esp_task_wdt_reset();
        if (!s_updating && millis() >= next_blink) {
            next_blink = millis() + 1000;
            EcoTrace::ledOn(); delay(30); EcoTrace::ledOff();
        }
        delay(10);
    }

    Serial.println("OTA mode: window closed -- resuming normal cycle");
    ArduinoOTA.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    EcoTrace::ledOff();
}

#else  /* no WiFi credentials */

void runWindow()
{
    Serial.println("OTA mode requested, but WIFI_SSID/WIFI_PASS are not set in secrets.h");
}

#endif

}  // namespace Ota
