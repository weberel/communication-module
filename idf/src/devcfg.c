#include "devcfg.h"
#include "config.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

static const char *TAG = "devcfg";

#define NVS_NS       "cfg"
#define KEY_VER      "ver"
#define KEY_SAMPLE   "samp_s"
#define KEY_UPLOAD   "upl_s"

/* Bounds from DOWNLINK.md. Re-checked here because the server UI is not the
 * only thing that can publish to the topic (the admin CLI can, and so could a
 * mistake), and an interval of 0 would keep the board awake until the battery
 * is flat. */
#define SAMPLE_MIN_S     60
#define SAMPLE_MAX_S   3600
#define UPLOAD_MIN_S    600
#define UPLOAD_MAX_S  86400

/* RTC cache. Valid across deep sleep and crash reboots; a cold boot reloads it
 * from NVS. The magic also changes when the layout does. */
#define CACHE_MAGIC 0xC0F16A01u
static RTC_DATA_ATTR uint32_t s_magic;
static RTC_DATA_ATTR uint32_t s_ver;
static RTC_DATA_ATTR uint32_t s_sample_s;
static RTC_DATA_ATTR uint32_t s_upload_s;
static RTC_DATA_ATTR uint8_t  s_err;      /* 0 none, else a devcfg_rc_t */

static bool s_nvs_ready;

static void cache_defaults(void)
{
    s_ver      = 0;
    s_sample_s = SAMPLE_INTERVAL_S;
    s_upload_s = UPLOAD_PERIOD_S;
    s_err      = 0;
}

static void load_from_nvs(void)
{
    cache_defaults();
    nvs_handle_t h;
    if (!s_nvs_ready || nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored config -- compiled defaults (%lu s / %lu s)",
                 (unsigned long)s_sample_s, (unsigned long)s_upload_s);
        return;
    }
    uint32_t v;
    if (nvs_get_u32(h, KEY_VER,    &v) == ESP_OK) s_ver = v;
    if (nvs_get_u32(h, KEY_SAMPLE, &v) == ESP_OK && v >= SAMPLE_MIN_S && v <= SAMPLE_MAX_S)
        s_sample_s = v;
    if (nvs_get_u32(h, KEY_UPLOAD, &v) == ESP_OK && v >= UPLOAD_MIN_S && v <= UPLOAD_MAX_S)
        s_upload_s = v;
    nvs_close(h);
    ESP_LOGI(TAG, "stored config ver %lu: sample %lu s, upload %lu s",
             (unsigned long)s_ver, (unsigned long)s_sample_s, (unsigned long)s_upload_s);
}

void devcfg_init(void)
{
    /* The standard dance: a partition left by another NVS version, or one
     * with no free pages, is erased and re-initialised. The only data in it
     * are these three keys and the WiFi calibration, both recoverable. */
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    s_nvs_ready = (e == ESP_OK);
    if (!s_nvs_ready) ESP_LOGE(TAG, "NVS init failed (%s)", esp_err_to_name(e));

    if (s_magic != CACHE_MAGIC) {
        load_from_nvs();
        s_magic = CACHE_MAGIC;
    }
}

uint32_t devcfg_sample_s(void) { return s_magic == CACHE_MAGIC ? s_sample_s : SAMPLE_INTERVAL_S; }
uint32_t devcfg_upload_s(void) { return s_magic == CACHE_MAGIC ? s_upload_s : UPLOAD_PERIOD_S; }
uint32_t devcfg_ver(void)      { return s_magic == CACHE_MAGIC ? s_ver : 0; }

const char *devcfg_err(void)
{
    switch ((devcfg_rc_t)s_err) {
    case DEVCFG_PARSE: return "parse";
    case DEVCFG_RANGE: return "range";
    default:           return NULL;
    }
}

const char *devcfg_rc_name(devcfg_rc_t rc)
{
    switch (rc) {
    case DEVCFG_APPLIED: return "applied";
    case DEVCFG_SAME:    return "same";
    case DEVCFG_EMPTY:   return "empty";
    case DEVCFG_PARSE:   return "parse";
    case DEVCFG_RANGE:   return "range";
    case DEVCFG_NVS:     return "nvs";
    }
    return "?";
}

/* A JSON number that is a non-negative integer; false for anything else.
 * An absent key is fine ("leave it") and reports present = false. */
static bool get_uint(const cJSON *o, const char *key, bool *present, uint32_t *out)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(o, key);
    *present = (n != NULL);
    if (!n) return true;
    if (!cJSON_IsNumber(n)) return false;
    double d = n->valuedouble;
    if (d < 0 || d > 4294967295.0 || d != (double)(uint32_t)d) return false;
    *out = (uint32_t)d;
    return true;
}

devcfg_rc_t devcfg_apply(const char *json, size_t len)
{
    if (len == 0) return DEVCFG_EMPTY;

    /* Bounded parse: the broker hands us data_len, not a C string. */
    cJSON *root = cJSON_ParseWithLength(json, len);
    devcfg_rc_t rc;
    if (!root || !cJSON_IsObject(root)) {
        rc = DEVCFG_PARSE;
        goto out;
    }

    bool has_ver, has_s, has_u;
    uint32_t ver = 0, sample = s_sample_s, upload = s_upload_s;

    if (!get_uint(root, "cfg_ver", &has_ver, &ver) || !has_ver || ver == 0 ||
        !get_uint(root, "sample_interval_s", &has_s, &sample) ||
        !get_uint(root, "upload_period_s",   &has_u, &upload)) {
        rc = DEVCFG_PARSE;
        goto out;
    }
    /* The whole payload stands or falls together; a partial apply would leave
     * the board on a cadence nobody asked for. Bounds are checked on the
     * resulting pair, so "upload >= sample" also holds when only one changed. */
    if (sample < SAMPLE_MIN_S || sample > SAMPLE_MAX_S ||
        upload < UPLOAD_MIN_S || upload > UPLOAD_MAX_S || upload < sample) {
        rc = DEVCFG_RANGE;
        goto out;
    }
    if (ver == s_ver) {
        s_err = 0;                             /* the last payload is fine */
        rc = DEVCFG_SAME;
        goto out;
    }

    /* Differs, not greater: DOWNLINK.md. Store first, then switch the cache,
     * so a power cut between the two leaves the old version in force and the
     * payload is re-applied next session. */
    nvs_handle_t h;
    if (!s_nvs_ready || nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        rc = DEVCFG_NVS;
        goto out;
    }
    esp_err_t e = nvs_set_u32(h, KEY_SAMPLE, sample);
    if (e == ESP_OK) e = nvs_set_u32(h, KEY_UPLOAD, upload);
    if (e == ESP_OK) e = nvs_set_u32(h, KEY_VER, ver);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed (%s)", esp_err_to_name(e));
        rc = DEVCFG_NVS;
        goto out;
    }
    s_sample_s = sample;
    s_upload_s = upload;
    s_ver      = ver;
    s_err      = 0;
    ESP_LOGI(TAG, "applied cfg_ver %lu: sample %lu s, upload %lu s",
             (unsigned long)ver, (unsigned long)sample, (unsigned long)upload);
    rc = DEVCFG_APPLIED;

out:
    cJSON_Delete(root);
    if (rc == DEVCFG_PARSE || rc == DEVCFG_RANGE) {
        s_err = (uint8_t)rc;
        ESP_LOGW(TAG, "payload rejected (%s), keeping ver %lu",
                 devcfg_rc_name(rc), (unsigned long)s_ver);
    }
    return rc;
}
