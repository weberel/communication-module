#include "ota.h"
#include "config.h"
#include "EcoTraceBoard.h"
#include "esp_task_wdt.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

/* OTA mode can use a different AP than the uplink (OTA_WIFI_* in secrets.h).
 * Needed when the site WLAN isolates clients -- eth-iot passes ICMP between
 * subnets but silently drops peer UDP, so espota never gets an answer there.
 * A PC mobile hotspot gives a direct link instead. Falls back to the uplink
 * credentials when no dedicated OTA AP is set. */
#if !defined(OTA_WIFI_SSID) && defined(WIFI_SSID)
#define OTA_WIFI_SSID WIFI_SSID
#define OTA_WIFI_PASS WIFI_PASS
#endif

#if defined(OTA_WIFI_SSID) && defined(OTA_WIFI_PASS)
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#endif

namespace Ota {

#if defined(OTA_WIFI_SSID) && defined(OTA_WIFI_PASS)

static volatile bool s_updating = false;

/* Second OTA path over plain TCP: POST the image to http://<ip>/update
 * (curl -F "image=@firmware.bin" ...). Exists because espota's UDP handshake
 * proved fragile across real-world APs/firewalls, and a browser/curl upload is
 * diagnosable: connection-refused vs timeout vs 200 each mean something. */
static WebServer s_http(80);

static void httpOtaBegin()
{
    s_http.on("/", HTTP_GET, []() {
        s_http.send(200, "text/plain",
                    String("ecoTrace ") + FW_VERSION + ", POST firmware to /update\n");
    });
    s_http.on("/update", HTTP_POST,
        []() {   /* request done -> report and reboot into the new image */
            bool ok = !Update.hasError();
            s_http.send(ok ? 200 : 500, "text/plain", ok ? "OK, rebooting\n" : "update failed\n");
            if (ok) { delay(300); ESP.restart(); }
        },
        []() {   /* streamed upload chunks */
            HTTPUpload& up = s_http.upload();
            if (up.status == UPLOAD_FILE_START) {
                s_updating = true;
                EcoTrace::ledOn();
                Serial.printf("HTTP OTA: receiving %s\n", up.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
            } else if (up.status == UPLOAD_FILE_WRITE) {
                esp_task_wdt_reset();
                if (Update.write(up.buf, up.currentSize) != up.currentSize)
                    Update.printError(Serial);
            } else if (up.status == UPLOAD_FILE_END) {
                if (Update.end(true)) Serial.printf("HTTP OTA: %u bytes, verified\n", up.totalSize);
                else                  Update.printError(Serial);
                s_updating = false;
            } else if (up.status == UPLOAD_FILE_ABORTED) {
                Update.abort();
                s_updating = false;
            }
        });
    s_http.begin();
}

void runWindow()
{
    Serial.printf("OTA mode: joining '%s'...\n", OTA_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   /* modem power-save drops/delays unicast (~0.5 s RTT,
                             * packet loss) and espota invitations die -- keep the
                             * radio awake for the whole OTA window */
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);
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
    httpOtaBegin();

    Serial.printf("OTA mode: listening as %s.local (%s) for %d s\n"
                  "  push with: pio run -e datalogger_ota -t upload\n"
                  "  or:  curl -F \"image=@firmware.bin\" http://%s/update\n",
                  OTA_HOSTNAME, WiFi.localIP().toString().c_str(), OTA_WINDOW_S,
                  WiFi.localIP().toString().c_str());

    /* Wait out the window (or however long an in-flight update needs). A short
     * LED blip every second says "I'm in OTA mode". */
    uint32_t deadline = millis() + (uint32_t)OTA_WINDOW_S * 1000;
    uint32_t next_blink = 0;
    while ((int32_t)(deadline - millis()) > 0 || s_updating) {
        ArduinoOTA.handle();
        s_http.handleClient();
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
    Serial.println("OTA mode requested, but no WiFi credentials are set in secrets.h");
}

#endif

}  // namespace Ota
