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
/* stage 2 */
#define KEY_TRY_VER  "fw_try_v"
#define KEY_TRY_N    "fw_try_n"
#define KEY_TRY_BY   "fw_try_by"   /* running version that made those tries */
#define KEY_ERR      "fw_err"
#define KEY_ERR_VER  "fw_err_v"

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

/* OTA state: plain statics. The target lives only for the session that
 * received it; tries and the last error are read from NVS on first use in a
 * boot (only upload sessions need them, and those are minutes long). */
static devcfg_fw_t s_fw;
static bool        s_fw_present;

static bool    s_fws_loaded;
static char    s_try_ver[32];
static char    s_try_by[32];
static uint8_t s_try_n;
static char    s_err_word[12];
static char    s_err_ver[32];

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
     * are these keys and the WiFi calibration, all recoverable. */
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

void devcfg_drop_cache(void) { s_magic = 0; }

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

/* A JSON string that fits; absent or wrong returns false. */
static bool get_str(const cJSON *o, const char *key, char *out, size_t cap)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(o, key);
    if (!n || !cJSON_IsString(n) || !n->valuestring) return false;
    size_t l = strlen(n->valuestring);
    if (l == 0 || l >= cap) return false;
    memcpy(out, n->valuestring, l + 1);
    return true;
}

/* The fw_* triple travels together or not at all (DOWNLINK.md). Anything
 * partial is treated as absent and said so, never acted on. */
static void take_fw_target(const cJSON *root)
{
    devcfg_fw_t t;
    int n = get_str(root, "fw_ver",    t.ver,    sizeof t.ver)
          + get_str(root, "fw_url",    t.url,    sizeof t.url)
          + get_str(root, "fw_sha256", t.sha256, sizeof t.sha256);
    if (n == 3 && strlen(t.sha256) == 64) {
        s_fw = t;
        s_fw_present = true;
        ESP_LOGI(TAG, "OTA target %s", t.ver);
    } else if (n != 0) {
        ESP_LOGW(TAG, "fw_* keys incomplete (%d of 3) -- ignored", n);
    }
}

devcfg_rc_t devcfg_apply(const char *json, size_t len)
{
    s_fw_present = false;                      /* the payload is total */
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
    take_fw_target(root);                      /* accepted payload from here on */
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

/* ---- OTA target and bookkeeping ------------------------------------------ */

const devcfg_fw_t *devcfg_fw_target(void) { return s_fw_present ? &s_fw : NULL; }

static void fw_state_load(void)
{
    if (s_fws_loaded) return;
    s_fws_loaded = true;
    s_try_ver[0] = s_err_word[0] = s_err_ver[0] = 0;
    s_try_n = 0;
    nvs_handle_t h;
    if (!s_nvs_ready || nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t l;
    l = sizeof s_try_ver;  if (nvs_get_str(h, KEY_TRY_VER, s_try_ver,  &l) != ESP_OK) s_try_ver[0]  = 0;
    l = sizeof s_try_by;   if (nvs_get_str(h, KEY_TRY_BY,  s_try_by,   &l) != ESP_OK) s_try_by[0]   = 0;
    l = sizeof s_err_word; if (nvs_get_str(h, KEY_ERR,     s_err_word, &l) != ESP_OK) s_err_word[0] = 0;
    l = sizeof s_err_ver;  if (nvs_get_str(h, KEY_ERR_VER, s_err_ver,  &l) != ESP_OK) s_err_ver[0]  = 0;
    if (nvs_get_u8(h, KEY_TRY_N, &s_try_n) != ESP_OK) s_try_n = 0;
    nvs_close(h);
    /* Attempts made by another running image (a serial reflash since, most
     * likely a fix) do not count against this one. */
    if (s_try_n && strcmp(s_try_by, FW_VERSION) != 0) {
        ESP_LOGI(TAG, "OTA attempts for %s were made by %s -- counter reset", s_try_ver, s_try_by);
        s_try_n = 0;
    }
    if (s_err_word[0])
        ESP_LOGW(TAG, "last OTA error: %s (%s)", s_err_word, s_err_ver);
}

static void fw_state_save(void)
{
    nvs_handle_t h;
    if (!s_nvs_ready || nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "cannot save OTA state");
        return;
    }
    nvs_set_str(h, KEY_TRY_VER, s_try_ver);
    nvs_set_str(h, KEY_TRY_BY,  FW_VERSION);
    nvs_set_u8 (h, KEY_TRY_N,   s_try_n);
    nvs_set_str(h, KEY_ERR,     s_err_word);
    nvs_set_str(h, KEY_ERR_VER, s_err_ver);
    nvs_commit(h);
    nvs_close(h);
}

static void copy_str(char *dst, size_t cap, const char *src)
{
    size_t l = src ? strlen(src) : 0;
    if (l >= cap) l = cap - 1;
    memcpy(dst, src, l);
    dst[l] = 0;
}

uint8_t devcfg_fw_tries(const char *ver)
{
    fw_state_load();
    return strcmp(s_try_ver, ver) == 0 ? s_try_n : 0;
}

void devcfg_fw_note_try(const char *ver)
{
    fw_state_load();
    if (strcmp(s_try_ver, ver) != 0) {
        copy_str(s_try_ver, sizeof s_try_ver, ver);
        s_try_n = 0;
        /* An error about some other version is history now. */
        if (strcmp(s_err_ver, ver) != 0) s_err_word[0] = s_err_ver[0] = 0;
    }
    if (s_try_n < 255) s_try_n++;
    fw_state_save();
    ESP_LOGI(TAG, "OTA attempt %u for %s", s_try_n, ver);
}

const char *devcfg_fw_err(void)     { fw_state_load(); return s_err_word[0] ? s_err_word : NULL; }
const char *devcfg_fw_err_ver(void) { fw_state_load(); return s_err_ver[0]  ? s_err_ver  : NULL; }

void devcfg_fw_set_err(const char *err, const char *ver)
{
    fw_state_load();
    copy_str(s_err_word, sizeof s_err_word, err);
    copy_str(s_err_ver,  sizeof s_err_ver,  ver);
    fw_state_save();
}

void devcfg_fw_clear_err(void)
{
    fw_state_load();
    if (!s_err_word[0]) return;
    s_err_word[0] = s_err_ver[0] = 0;
    fw_state_save();
}

bool devcfg_fw_note_rollback(const char *ver)
{
    fw_state_load();
    if (strcmp(s_err_word, "rollback") == 0 && strcmp(s_err_ver, ver) == 0)
        return false;                          /* already on record */
    devcfg_fw_set_err("rollback", ver);
    return true;
}
