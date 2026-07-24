#include "ModemA7672.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>

/* ===================== low-level helpers ===================== */
void ModemA7672::flushRx()
{
    while (_uart->available()) _uart->read();
}

void ModemA7672::pwrkeyPulse(uint32_t ms)
{
    digitalWrite(ECO_PIN_MODEM_PWRKEY, LOW);
    delay(ms);
    digitalWrite(ECO_PIN_MODEM_PWRKEY, HIGH);
}

bool ModemA7672::statusHigh()
{
    return digitalRead(ECO_PIN_MODEM_STATUS) == HIGH;
}

int ModemA7672::waitStatus(int level, uint32_t timeout_ms)
{
    uint32_t n = timeout_ms / 100;
    for (uint32_t i = 0; i < n; i++) {
        delay(100);
        if (digitalRead(ECO_PIN_MODEM_STATUS) == level) return (int)((i + 1) * 100);
    }
    return -1;
}

bool ModemA7672::waitFor(const char* needle, uint32_t timeout_ms)
{
    memset(_resp, 0, sizeof(_resp));
    size_t got = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms && got < sizeof(_resp) - 1) {
        while (_uart->available() && got < sizeof(_resp) - 1)
            _resp[got++] = (char)_uart->read();
        if (strstr(_resp, needle)) return true;
        delay(5);
    }
    return false;
}

bool ModemA7672::sendAT(const char* cmd, const char* expect, uint32_t timeout_ms)
{
    flushRx();
    _uart->print(cmd);
    _uart->print("\r\n");

    memset(_resp, 0, sizeof(_resp));
    size_t got = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms && got < sizeof(_resp) - 1) {
        while (_uart->available() && got < sizeof(_resp) - 1)
            _resp[got++] = (char)_uart->read();
        if (strstr(_resp, expect))  return true;
        if (strstr(_resp, "ERROR")) return false;
        delay(5);
    }
    return false;
}

/* ===================== power on / off ===================== */
bool ModemA7672::begin(uint32_t baud)
{
    pinMode(ECO_PIN_MODEM_STATUS, INPUT_PULLUP);
    pinMode(ECO_PIN_MODEM_PWRKEY, OUTPUT);
    digitalWrite(ECO_PIN_MODEM_PWRKEY, HIGH);       /* idle high */
    pinMode(ECO_PIN_MODEM_PWR_EN, OUTPUT);

    /* Apply the modem rail. */
    digitalWrite(ECO_PIN_MODEM_PWR_EN, HIGH);
    delay(200);

    /* Boot pulse: PWRKEY low ~600 ms. */
    pwrkeyPulse(600);

    /* Wait for STATUS to assert (modem powered). */
    if (waitStatus(HIGH, 10000) < 0) {
        digitalWrite(ECO_PIN_MODEM_PWR_EN, LOW);
        return false;
    }

    _uart->begin(baud, SERIAL_8N1, ECO_PIN_MODEM_RX, ECO_PIN_MODEM_TX);

    /* Poll AT until it answers (the firmware needs a second after STATUS). */
    for (int i = 0; i < 15; i++) {
        if (sendAT("AT", "OK", 1000)) {
            sendAT("ATE0", "OK", 1000);   /* echo off, cleaner parsing */
            return true;
        }
        delay(500);
    }
    return false;
}

void ModemA7672::powerOff()
{
    sendAT("AT+CPOF", "OK", 2000);
    if (waitStatus(LOW, 20000) < 0) {
        pwrkeyPulse(3500);              /* long pulse fallback */
        waitStatus(LOW, 15000);
    }
    digitalWrite(ECO_PIN_MODEM_PWR_EN, LOW);   /* force rail off regardless */
}

/* ===================== info / SIM / network ===================== */
bool ModemA7672::getInfo(char* out, size_t len)
{
    if (!sendAT("ATI", "OK", 2000)) return false;
    strncpy(out, _resp, len - 1);
    out[len - 1] = 0;
    for (size_t i = 0; i < strlen(out); i++)
        if (out[i] == '\r' || out[i] == '\n') out[i] = ' ';
    return true;
}

bool ModemA7672::simReady()
{
    flushRx();
    _uart->print("AT+CPIN?\r\n");
    if (!waitFor("+CPIN:", 5000)) return false;
    return strstr(_resp, "READY") != nullptr;
}

bool ModemA7672::waitForNetwork(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
        if (sendAT("AT+CREG?", "+CREG:", 2000)) {
            char* p = strstr(_resp, "+CREG:");
            int n, stat;
            if (p && sscanf(p, "+CREG: %d,%d", &n, &stat) == 2 && (stat == 1 || stat == 5))
                return true;
        }
        delay(1000);
    }
    return false;
}

int ModemA7672::signalQuality_dBm()
{
    if (!sendAT("AT+CSQ", "+CSQ:", 2000)) return 0;
    char* p = strstr(_resp, "+CSQ:");
    int rssi = 99, ber = 0;
    if (p && sscanf(p, "+CSQ: %d,%d", &rssi, &ber) == 2 && rssi != 99)
        return -113 + rssi * 2;
    return 0;
}

int ModemA7672::batteryVoltage_mV()
{
    if (!sendAT("AT+CBC", "+CBC:", 2000)) return 0;
    char* p = strstr(_resp, "+CBC:");
    int bcs, bcl, mv;
    if (p && sscanf(p, "+CBC: %d,%d,%d", &bcs, &bcl, &mv) == 3) return mv;
    return 0;
}

/* ===================== data / HTTP ===================== */
bool ModemA7672::connectGPRS(const char* apn)
{
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    sendAT(cmd, "OK", 2000);
    return sendAT("AT+CGACT=1,1", "OK", 30000);
}

bool ModemA7672::httpGet(const char* url, int& status, char* body, size_t body_len)
{
    status = -1;
    sendAT("AT+HTTPTERM", "OK", 1000);
    if (!sendAT("AT+HTTPINIT", "OK", 5000)) return false;

    sendAT("AT+HTTPPARA=\"CID\",1", "OK", 2000);
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (!sendAT(cmd, "OK", 2000)) { sendAT("AT+HTTPTERM", "OK", 1000); return false; }

    bool ok = false;
    if (sendAT("AT+HTTPACTION=0", "OK", 5000) && waitFor("+HTTPACTION:", 60000)) {
        char* p = strstr(_resp, "+HTTPACTION:");
        int method, len;
        if (p && sscanf(p, "+HTTPACTION: %d,%d,%d", &method, &status, &len) == 3)
            ok = (status >= 200 && status < 300);
    }
    if (ok && body && body_len) {
        if (sendAT("AT+HTTPREAD=0,512", "+HTTPREAD:", 5000)) {
            strncpy(body, _resp, body_len - 1);
            body[body_len - 1] = 0;
        }
    }
    sendAT("AT+HTTPTERM", "OK", 2000);
    return ok;
}

bool ModemA7672::httpPost(const char* url, const char* content_type, const char* payload,
                          int& status, char* body, size_t body_len)
{
    status = -1;
    sendAT("AT+HTTPTERM", "OK", 1000);
    if (!sendAT("AT+HTTPINIT", "OK", 5000)) return false;

    sendAT("AT+HTTPPARA=\"CID\",1", "OK", 2000);
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (!sendAT(cmd, "OK", 2000)) { sendAT("AT+HTTPTERM", "OK", 1000); return false; }
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"CONTENT\",\"%s\"", content_type);
    sendAT(cmd, "OK", 2000);

    /* Hand the modem the body: AT+HTTPDATA=<len>,<timeout_ms> -> "DOWNLOAD" prompt. */
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,10000", (int)strlen(payload));
    if (!sendAT(cmd, "DOWNLOAD", 5000)) { sendAT("AT+HTTPTERM", "OK", 1000); return false; }
    _uart->write((const uint8_t*)payload, strlen(payload));
    if (!waitFor("OK", 10000)) { sendAT("AT+HTTPTERM", "OK", 1000); return false; }

    bool ok = false;
    if (sendAT("AT+HTTPACTION=1", "OK", 5000) && waitFor("+HTTPACTION:", 60000)) {
        char* p = strstr(_resp, "+HTTPACTION:");
        int method, len;
        if (p && sscanf(p, "+HTTPACTION: %d,%d,%d", &method, &status, &len) == 3)
            ok = (status >= 200 && status < 300);
    }
    if (ok && body && body_len) {
        if (sendAT("AT+HTTPREAD=0,512", "+HTTPREAD:", 5000)) {
            strncpy(body, _resp, body_len - 1);
            body[body_len - 1] = 0;
        }
    }
    sendAT("AT+HTTPTERM", "OK", 2000);
    return ok;
}

/* ===================== time ===================== */
int64_t ModemA7672::getUnixTimeMs()
{
    if (!sendAT("AT+CCLK?", "+CCLK:", 3000)) return 0;
    char* p = strstr(_resp, "+CCLK: \"");
    if (!p) return 0;
    p += 8;
    int yy, mo, dd, hh, mm, ss, tz = 0;
    char sign = '+';
    if (sscanf(p, "%d/%d/%d,%d:%d:%d%c%d", &yy, &mo, &dd, &hh, &mm, &ss, &sign, &tz) < 6)
        return 0;

    struct tm t = {};
    t.tm_year = yy + 100;   /* 2-digit year -> years since 1900 */
    t.tm_mon  = mo - 1;
    t.tm_mday = dd;
    t.tm_hour = hh;
    t.tm_min  = mm;
    t.tm_sec  = ss;
    setenv("TZ", "UTC", 1);
    tzset();
    time_t unix_sec = mktime(&t);
    int offset_s = tz * 15 * 60;                 /* TZ in quarter-hours */
    unix_sec += (sign == '+') ? -offset_s : offset_s;
    return (int64_t)unix_sec * 1000;
}

/* ===================== GPS (stub -- untested hardware) ===================== */
bool ModemA7672::gpsEnable()
{
    /* TODO: validate on hardware. Likely AT+CGNSSPWR=1 then wait for +CGNSSPWR: READY.
     * The GPS antenna path on Rev A is unverified -- see docs/hardware-errata.md. */
    return false;
}

bool ModemA7672::gpsGetFix(float& lat, float& lon, uint32_t timeout_ms)
{
    (void)lat; (void)lon; (void)timeout_ms;
    /* TODO: AT+CGNSSINFO, parse the fix. Not implemented until GPS is validated. */
    return false;
}
