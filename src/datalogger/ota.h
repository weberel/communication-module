/*
 * ota.h  --  WiFi OTA mode (bench/testing convenience).
 *
 * Entered by pressing the QON button BUTTON_OTA_PRESSES times (see config.h).
 * Joins WIFI_SSID, announces itself as OTA_HOSTNAME via mDNS, and accepts an
 * ArduinoOTA firmware push for OTA_WINDOW_S:
 *
 *     pio run -e datalogger_ota -t upload
 *
 * On success the board reboots into the new image (dual ota_0/ota_1 slots, see
 * partitions.csv); on timeout it resumes the normal sample cycle. There is NO
 * automatic rollback in the Arduino build -- a broken image means recovering
 * over USB. Real field OTA (cellular, rollback-safe) belongs to the ESP-IDF port.
 */
#pragma once

namespace Ota {

/* Block in OTA mode until a push completes (device reboots) or the window
 * expires (returns). Safe to call before the charger/flash are initialised. */
void runWindow();

}  // namespace Ota
