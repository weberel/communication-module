/*
 * ModemA7672.h  --  driver for the SIMCom A7672E LTE Cat-1 modem
 * =============================================================
 * The board is fitted with an A7672E-LASE (confirmed via ATI). It hangs off UART1
 * with a MOSFET power rail (GPIO23) and a PWRKEY line (GPIO22); the modem's own
 * STATUS output (GPIO0) reports whether it is powered.
 *
 * This is a deliberately hand-rolled AT driver rather than a library like TinyGSM,
 * because the A7672E has enough quirks (boot timing, HTTP AT flow, power sequencing)
 * that precise control is worth more than the abstraction. sendAT()/waitFor() and
 * the exposed uart() Stream let you drive it directly for anything not wrapped here.
 *
 * Typical use:
 *     ModemA7672 modem;
 *     if (modem.begin()) {
 *         modem.waitForNetwork(30000);
 *         modem.connectGPRS("internet");
 *         int status; modem.httpGet("http://example.com/", status);
 *         modem.powerOff();
 *     }
 */
#pragma once

#include <Arduino.h>
#include "ecotrace_pins.h"

class ModemA7672 {
public:
    explicit ModemA7672(HardwareSerial& uart = Serial1) : _uart(&uart) {}

    /* Power the modem rail, pulse PWRKEY, wait for STATUS high + AT/OK.
     * Returns false if the modem never boots or never answers AT. */
    bool begin(uint32_t baud = ECO_MODEM_BAUD);

    /* Graceful AT+CPOF, then drop the power rail. Falls back to a long PWRKEY pulse. */
    void powerOff();

    /* True once the modem STATUS pin is high (modem powered). */
    bool statusHigh();

    /* ---- Raw AT interface --------------------------------------------------- */
    /* Send "cmd\r\n" and wait until `expect` (default "OK") or "ERROR"/timeout.
     * The full response is captured in lastResponse(). */
    bool sendAT(const char* cmd, const char* expect = "OK", uint32_t timeout_ms = 2000);
    /* Wait for `needle` in the incoming stream without sending anything. */
    bool waitFor(const char* needle, uint32_t timeout_ms);
    const char* lastResponse() const { return _resp; }
    Stream& uart() { return *_uart; }

    /* ---- Info / SIM / network ----------------------------------------------- */
    bool getInfo(char* out, size_t len);   /* ATI */
    bool simReady();                        /* AT+CPIN? -> READY */
    bool waitForNetwork(uint32_t timeout_ms = 30000);   /* CREG home(1)/roaming(5) */
    int  signalQuality_dBm();               /* AT+CSQ -> dBm, or 0 if unknown */
    int  batteryVoltage_mV();               /* AT+CBC (modem's own VBAT sense) */

    /* ---- Data / HTTP -------------------------------------------------------- */
    bool connectGPRS(const char* apn);      /* CGDCONT + CGACT */
    /* HTTP GET/POST via the modem's AT+HTTP* stack. Returns true on a 2xx and
     * writes the HTTP status into `status`. Optional response body capture. */
    bool httpGet(const char* url, int& status,
                 char* body = nullptr, size_t body_len = 0);
    bool httpPost(const char* url, const char* content_type, const char* payload,
                  int& status, char* body = nullptr, size_t body_len = 0);

    /* ---- Time --------------------------------------------------------------- */
    /* AT+CCLK? parsed to Unix ms (requires NITZ/network time). 0 on failure.
     * CAUTION: without NITZ the modem reports its default 1970 epoch, which the
     * 2-digit-year parse renders as 2070 -- sanity-check the result, and use
     * ntpSync() first on carriers that do not send network time. */
    int64_t getUnixTimeMs();

    /* Sync the modem clock via NTP over the data connection (AT+CNTP; needs
     * connectGPRS first). After success getUnixTimeMs() returns real time even
     * on carriers without NITZ. */
    bool ntpSync(const char* server = "pool.ntp.org");

    /* ---- GPS (UNTESTED on Rev A hardware -- see docs/hardware-errata.md) ----- */
    /* Left as a stub on purpose. The A7672E-LASE has an internal GNSS engine
     * (AT+CGNSSPWR / AT+CGNSSINFO) but it has not been validated on this board and
     * the GPS antenna path is unverified. Implement + test before relying on it. */
    bool gpsEnable();
    bool gpsGetFix(float& lat, float& lon, uint32_t timeout_ms = 60000);

private:
    HardwareSerial* _uart;
    char _resp[512];

    void pwrkeyPulse(uint32_t ms);
    int  waitStatus(int level, uint32_t timeout_ms);
    void flushRx();
};
