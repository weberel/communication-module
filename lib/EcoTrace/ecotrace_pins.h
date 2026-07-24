/*
 * ecotrace_pins.h  --  ecoTrace Communication Module pin map (SINGLE SOURCE OF TRUTH)
 * ==================================================================================
 * Board : ecoTrace Communication Module, Rev A (2026-04-23)
 * MCU   : ESP32-C6-MINI-1
 *
 * Verified against schematic 2026-05-05. If the board is respun, this file is the
 * ONE place the firmware pin map changes.
 *
 * Exposed GPIOs on the MINI-1 module:
 *   0,1,2,3,4,5,6,7,8,9,12,13,14,15,18,19,20,21,22,23
 *   (GPIO10,11,16,17 not bonded on MINI-1; GPIO24-30 are internal flash)
 */
#pragma once

/* ---- Status / user LED ------------------------------------------------------ */
#define ECO_PIN_LED            1     /* general status LED (active HIGH) */

/* ---- I2C (LP_I2C, external 4k7 pull-ups to 3V3) ----------------------------- */
#define ECO_PIN_I2C_SDA        6
#define ECO_PIN_I2C_SCL        7
#define ECO_I2C_FREQ_HZ        100000

/* ---- SPI (external GD25Q256 NOR flash + one spare CS) ----------------------- */
#define ECO_PIN_SPI_MOSI       4     /* SI  */
#define ECO_PIN_SPI_MISO       5     /* SO  */
#define ECO_PIN_SPI_CLK        18
#define ECO_PIN_SPI_CS_FLASH   8     /* GD25Q256; strapping pin, idles HIGH via 10k */
#define ECO_PIN_SPI_CS_PERIPH  9     /* NOT POPULATED; strapping + BOOT pad, idles HIGH */

/* ---- A7672E LTE modem (UART1) ----------------------------------------------- */
#define ECO_PIN_MODEM_TX       20    /* ESP TX  -> modem RX */
#define ECO_PIN_MODEM_RX       21    /* ESP RX  <- modem TX */
#define ECO_PIN_MODEM_PWRKEY   22    /* push-pull, idle HIGH; LOW 0.5-1s = on, >=3s = off */
#define ECO_PIN_MODEM_PWR_EN   23    /* drives NPN -> P-MOSFET; HIGH = modem rail on */
#define ECO_PIN_MODEM_STATUS   0     /* modem STATUS out; HIGH = powered. NOTE: strapping pin */
#define ECO_MODEM_BAUD         115200

/* ---- Wake / interrupt inputs (all have external pull-ups, idle HIGH) -------- */
#define ECO_PIN_QON            2     /* BQ25792 QON; pull LOW to wake. DO NOT hold LOW > 2 s */
#define ECO_PIN_INT_SHARED     3     /* shared INT: SC7A20 accel + LTR-303 light */
#define ECO_PIN_INT_BQ         19    /* BQ25792 fault / charge interrupt */

/* ---- External sensor power rail --------------------------------------------- */
/* GPIO14: high-side switch (NPN -> P-MOSFET), active HIGH = external sensor rail on.
 * POPULATION VARIES BY BOARD -- confirm on yours before relying on it.
 *   - Some boards ship this footprint UNpopulated (sensors then wired direct to 3V3).
 *   - The on-board SC7A20 + LTR-303 are always wired to 3V3, NOT this rail; they can
 *     only be low-power'd over I2C. See docs/hardware-errata.md. */
#define ECO_PIN_SENSOR_PWR     14
#define ECO_SENSOR_PWR_ON      HIGH
#define ECO_SENSOR_PWR_OFF     LOW

/* ---- Strapping pins (left in safe states by hardware) ------------------------ */
/* GPIO8  CS_flash     -> idles HIGH via 10k pull-up
 * GPIO9  CS_periph    -> idles HIGH via 10k pull-up (also BOOT pad on back)
 * GPIO15 -> tied to GND
 * GPIO0  modem STATUS -> read briefly only; it is also a boot strap */

/* ---- I2C device addresses (7-bit) ------------------------------------------- */
#define ECO_ADDR_SC7A20_LO     0x18  /* accelerometer (SA0 low) */
#define ECO_ADDR_SC7A20_HI     0x19  /* accelerometer (SA0 high) */
#define ECO_ADDR_LTR303        0x29  /* ambient light sensor */
#define ECO_ADDR_ATECC608B_A   0x60  /* secure element (default) */
#define ECO_ADDR_ATECC608B_B   0x35  /* secure element (alt) */
#define ECO_ADDR_BQ25792       0x6B  /* charger / PMIC */

/* ---- Debug / fixture pads (back side) --------------------------------------- */
/* TP401 = TXD0 (UART0 flash TX)      TP402 = RXD0 (UART0 flash RX)
 * TP403 = GPIO9 (CS_periph / BOOT)   TP404 = GPIO2 (QON / wake)
 * EN    = reset (10k to 3V3, 1uF to GND, button to GND) */
