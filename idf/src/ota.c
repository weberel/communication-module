#include "ota.h"
#include "devcfg.h"

#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"

static const char *TAG = "ota";

/* ISRG Root X1, the same pinned root the broker connection uses (isrg_cert.c).
 * The image host serves the chain cross-signed by X1 for exactly this reason. */
extern const char isrg_root_pem[];

#define OTA_HTTP_TIMEOUT_MS   30000
#define OTA_LOG_EVERY_BYTES   (128 * 1024)
/* A read timeout makes esp_https_ota_perform() return IN_PROGRESS, not an
 * error, so a stream that died (2026-09-30: modem bytes lost during a flash
 * write wedged PPP) would loop forever with the watchdog fed. Give up when
 * no byte has arrived for OTA_STALL_MS, and in any case after OTA_DEADLINE_MS
 * (1.3 MB at the modem's 115200 baud is ~3 min; this is 3x that). */
#define OTA_STALL_MS          60000
#define OTA_DEADLINE_MS       (9 * 60 * 1000)

/* Off the 4 KB main-task stack. One caller at a time (the upload session). */
static uint8_t s_buf[4096];

const char *ota_rc_name(ota_rc_t rc)
{
    switch (rc) {
    case OTA_OK:       return "ok";
    case OTA_DOWNLOAD: return "download";
    case OTA_SHA256:   return "sha256";
    case OTA_WRITE:    return "write";
    }
    return "?";
}

static bool hex_equal(const uint8_t *bin, size_t n, const char *hex)
{
    static const char digits[] = "0123456789abcdef";
    if (strlen(hex) != 2 * n) return false;
    for (size_t i = 0; i < n; i++) {
        if (digits[bin[i] >> 4]  != (char)tolower((unsigned char)hex[2 * i]) ||
            digits[bin[i] & 0xF] != (char)tolower((unsigned char)hex[2 * i + 1]))
            return false;
    }
    return true;
}

/* SHA-256 of the first len bytes of the partition: exactly the bytes of the
 * .bin file the server hashed, not the padded partition. */
static bool partition_sha256(const esp_partition_t *p, size_t len, uint8_t out[32])
{
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    bool ok = mbedtls_sha256_starts(&ctx, 0) == 0;
    for (size_t off = 0; ok && off < len; off += sizeof(s_buf)) {
        size_t n = len - off < sizeof(s_buf) ? len - off : sizeof(s_buf);
        ok = esp_partition_read(p, off, s_buf, n) == ESP_OK &&
             mbedtls_sha256_update(&ctx, s_buf, n) == 0;
        esp_task_wdt_reset();
    }
    if (ok) ok = mbedtls_sha256_finish(&ctx, out) == 0;
    mbedtls_sha256_free(&ctx);
    return ok;
}

/* Errors from the flash side of esp_https_ota_perform() are ESP_ERR_FLASH_* or
 * the OTA validation code; everything else is the network. */
static ota_rc_t classify_perform(esp_err_t e)
{
    if (e == ESP_ERR_OTA_VALIDATE_FAILED) return OTA_WRITE;
    if ((e & 0xFF00) == ESP_ERR_FLASH_BASE) return OTA_WRITE;
    return OTA_DOWNLOAD;
}

ota_rc_t ota_run(const char *ver, const char *url, const char *sha256_hex)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) { ESP_LOGE(TAG, "no update partition"); return OTA_WRITE; }
    ESP_LOGI(TAG, "%s -> %s (slot %s at 0x%lx)", ver, url, target->label,
             (unsigned long)target->address);

    esp_http_client_config_t http = {
        .url               = url,
        .cert_pem          = isrg_root_pem,
        .timeout_ms        = OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t cfg = { .http_config = &http };
    esp_https_ota_handle_t h = NULL;

    esp_err_t e = esp_https_ota_begin(&cfg, &h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "begin failed: %s", esp_err_to_name(e));
        return OTA_DOWNLOAD;
    }

    /* Logged, not enforced: the hash is the identity. A descriptor that
     * disagrees with the config will fail the hash too, with a clearer trail. */
    esp_app_desc_t desc;
    if (esp_https_ota_get_img_desc(h, &desc) == ESP_OK)
        ESP_LOGI(TAG, "image says %s %s (%s %s)%s", desc.project_name, desc.version,
                 desc.date, desc.time, strcmp(desc.version, ver) ? "  MISMATCH" : "");

    int total = esp_https_ota_get_image_size(h), last_log = 0, last_len = 0;
    int64_t t0 = esp_timer_get_time(), t_progress = t0;
    bool stalled = false;
    for (;;) {
        e = esp_https_ota_perform(h);
        esp_task_wdt_reset();
        if (e != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        int got = esp_https_ota_get_image_len_read(h);
        int64_t now = esp_timer_get_time();
        if (got != last_len) { last_len = got; t_progress = now; }
        if (got - last_log >= OTA_LOG_EVERY_BYTES) {
            ESP_LOGI(TAG, "%d / %d bytes, %lu s", got, total,
                     (unsigned long)((now - t0) / 1000000));
            last_log = got;
        }
        if (now - t_progress > (int64_t)OTA_STALL_MS * 1000 ||
            now - t0 > (int64_t)OTA_DEADLINE_MS * 1000) {
            stalled = true;
            break;
        }
    }
    int len = esp_https_ota_get_image_len_read(h);
    if (stalled) {
        ESP_LOGE(TAG, "stream stalled at %d of %d bytes after %lu s -- giving up", len, total,
                 (unsigned long)((esp_timer_get_time() - t0) / 1000000));
        esp_https_ota_abort(h);
        return OTA_DOWNLOAD;
    }
    if (e != ESP_OK) {
        ota_rc_t rc = classify_perform(e);
        ESP_LOGE(TAG, "perform failed after %d bytes: %s (%s)", len,
                 esp_err_to_name(e), ota_rc_name(rc));
        esp_https_ota_abort(h);
        return rc;
    }
    if (!esp_https_ota_is_complete_data_received(h)) {
        ESP_LOGE(TAG, "short read: %d of %d bytes", len, total);
        esp_https_ota_abort(h);
        return OTA_DOWNLOAD;
    }

    uint8_t sha[32];
    if (!partition_sha256(target, (size_t)len, sha) || !hex_equal(sha, 32, sha256_hex)) {
        ESP_LOGE(TAG, "SHA-256 mismatch over %d bytes -- not switching", len);
        esp_https_ota_abort(h);
        return OTA_SHA256;
    }

    /* Validates the image header and chain, then writes otadata: the new slot
     * is NEW, the bootloader makes it PENDING_VERIFY on the first boot. */
    e = esp_https_ota_finish(h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "finish failed: %s", esp_err_to_name(e));
        return OTA_WRITE;
    }
    ESP_LOGI(TAG, "%s written and verified (%d bytes), boots next", ver, len);
    return OTA_OK;
}

bool ota_boot_check(void)
{
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_ota_img_states_t st;
    if (!other || esp_ota_get_state_partition(other, &st) != ESP_OK) return false;
    if (st != ESP_OTA_IMG_ABORTED) return false;

    esp_app_desc_t d;
    const char *v = (esp_ota_get_partition_description(other, &d) == ESP_OK) ? d.version : "?";
    bool fresh = devcfg_fw_note_rollback(v);
    if (fresh) ESP_LOGW(TAG, "slot %s (%s) was rolled back by the bootloader", other->label, v);
    return fresh;
}
