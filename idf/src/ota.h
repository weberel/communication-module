/*
 * ota.h -- firmware update over the link that is already up (downlink stage 2,
 * com-0.36). Contract: ecotrace-server/docs/DOWNLINK.md, "OTA semantics".
 *
 * The server's retained config names a version, a URL and a SHA-256. After the
 * drain, uplink.c calls ota_run(): download into the inactive slot with the
 * IDF HTTPS OTA client (same pinned root as the broker), hash what was written,
 * and only then make it the boot partition. main.c restarts the board.
 *
 * Rollback is the bootloader's: the new image marks itself valid only at the
 * end of its first successful upload session (main.c). Reaching deep sleep
 * without that mark means the next wake boots the previous image, and
 * ota_boot_check() is how that image notices and reports it.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTA_OK = 0,       /* written, hash verified, boot partition switched: restart */
    OTA_DOWNLOAD,     /* could not fetch the image (TLS, HTTP, timeout, short read) */
    OTA_SHA256,       /* fetched, but the hash of what was written differs */
    OTA_WRITE,        /* the flash write or the image validation failed */
} ota_rc_t;

/* Blocking; a minute or so on cellular. Needs an IP link and the task WDT
 * subscribed on the calling task (it is reset inside the loop). */
ota_rc_t ota_run(const char *ver, const char *url, const char *sha256_hex);

/* The contract's fw_err word for a result. */
const char *ota_rc_name(ota_rc_t rc);

/* At boot: if the other slot holds an image the bootloader ABORTED (it never
 * marked itself valid), record fw_err "rollback" for that version in NVS.
 * Returns true the first time a given rollback is seen, so the caller can
 * force an upload session and report it now rather than at the next
 * scheduled one. */
bool ota_boot_check(void);

#ifdef __cplusplus
}
#endif
