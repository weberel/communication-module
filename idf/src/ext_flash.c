#include "ext_flash.h"

#include <string.h>
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* GD25Q128 command set (3-byte addressing) */
#define FCMD_WREN 0x06
#define FCMD_RDSR 0x05
#define FCMD_READ 0x03
#define FCMD_PP   0x02   /* page program */
#define FCMD_SE   0x20   /* 4 KB sector erase */
#define FCMD_RDID 0x9F
#define FCMD_DP   0xB9   /* deep power-down */
#define FCMD_RDP  0xAB   /* release deep power-down */

static spi_device_handle_t s_dev;
static bool s_bus_ready;

static void txn(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = { 0 };
    t.length    = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(s_dev, &t);
}

static void cmd1(uint8_t c) { txn(&c, NULL, 1); }

static uint8_t rdsr(void)
{
    uint8_t tx[2] = { FCMD_RDSR, 0 }, rx[2] = { 0 };
    txn(tx, rx, sizeof(tx));
    return rx[1];
}

static void wait_ready(void)
{
    int64_t t0 = esp_timer_get_time();
    while ((rdsr() & 0x01) && (esp_timer_get_time() - t0) < 2000000)   /* WIP bit */
        vTaskDelay(pdMS_TO_TICKS(1));
}

static bool bus_init(void)
{
    if (s_bus_ready) return true;

    spi_bus_config_t bus = {
        .mosi_io_num = EXTFLASH_PIN_MOSI,
        .miso_io_num = EXTFLASH_PIN_MISO,
        .sclk_io_num = EXTFLASH_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4 + EXTFLASH_PAGE_SIZE,
    };
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000,   /* same 8 MHz as the Arduino driver */
        .mode = 0,
        .spics_io_num = EXTFLASH_PIN_CS,
        .queue_size = 1,
    };
    if (spi_bus_add_device(SPI2_HOST, &dev, &s_dev) != ESP_OK) return false;

    s_bus_ready = true;
    return true;
}

uint32_t extflash_jedec_id(void)
{
    uint8_t tx[4] = { FCMD_RDID, 0, 0, 0 }, rx[4] = { 0 };
    txn(tx, rx, sizeof(tx));
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

bool extflash_begin(void)
{
    if (!bus_init()) return false;
    extflash_wake();
    return (extflash_jedec_id() >> 16) == 0xC8;   /* GigaDevice */
}

void extflash_read(uint32_t addr, uint8_t *buf, size_t n)
{
    /* header + payload in one transaction (n <= page size in all our uses) */
    uint8_t tx[4 + EXTFLASH_PAGE_SIZE] = { FCMD_READ,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    uint8_t rx[4 + EXTFLASH_PAGE_SIZE];
    txn(tx, rx, 4 + n);
    memcpy(buf, rx + 4, n);
}

void extflash_erase_sector(uint32_t addr)
{
    cmd1(FCMD_WREN);
    uint8_t tx[4] = { FCMD_SE,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    txn(tx, NULL, sizeof(tx));
    wait_ready();
}

void extflash_write_page(uint32_t addr, const uint8_t *buf, size_t n)
{
    cmd1(FCMD_WREN);
    uint8_t tx[4 + EXTFLASH_PAGE_SIZE] = { FCMD_PP,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    memcpy(tx + 4, buf, n);
    txn(tx, NULL, 4 + n);
    wait_ready();
}

void extflash_power_down(void) { cmd1(FCMD_DP); }

void extflash_wake(void)
{
    cmd1(FCMD_RDP);
    esp_rom_delay_us(50);
}
