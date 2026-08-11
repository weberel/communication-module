/*
 * ext_flash.h  --  GD25Q128 SPI NOR driver, ESP-IDF spi_master port.
 * Same command set and semantics as the validated Arduino driver
 * (lib/EcoTrace/ExtFlash.*): 3-byte addressing, erase-before-write,
 * <=256 B page programs, deep power-down between wakes.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXTFLASH_SECTOR_SIZE 4096u
#define EXTFLASH_PAGE_SIZE   256u

/* Pin map, single source of truth = lib/EcoTrace/ecotrace_pins.h (Rev A).
 * Overridable via build flags for unit B's bypass rework (CS=1, CLK=9):
 *   build_flags = -DEXTFLASH_PIN_CS=1 -DEXTFLASH_PIN_CLK=9 */
#ifndef EXTFLASH_PIN_MOSI
#define EXTFLASH_PIN_MOSI 4
#endif
#ifndef EXTFLASH_PIN_MISO
#define EXTFLASH_PIN_MISO 5
#endif
#ifndef EXTFLASH_PIN_CLK
#define EXTFLASH_PIN_CLK  18
#endif
#ifndef EXTFLASH_PIN_CS
#define EXTFLASH_PIN_CS   8
#endif

/* Init SPI bus + device, wake from deep power-down, verify GigaDevice JEDEC. */
bool     extflash_begin(void);
uint32_t extflash_jedec_id(void);

void extflash_read(uint32_t addr, uint8_t *buf, size_t n);
void extflash_erase_sector(uint32_t addr);                       /* 4 KB */
void extflash_write_page(uint32_t addr, const uint8_t *buf, size_t n);

void extflash_power_down(void);
void extflash_wake(void);

#ifdef __cplusplus
}
#endif
