/*
 * EcoTraceBoard.h  --  board-level init and helpers for the ecoTrace Communication Module
 * ---------------------------------------------------------------------------------------
 * Brings the board up in a SAFE state (modem rail off, sensor rail off, strapping
 * pins untouched), starts the I2C and SPI buses on the mapped pins, and offers a
 * deep-sleep helper that holds the power rails off through sleep.
 *
 * This is deliberately thin: it owns the pins and the buses, nothing else. The
 * BQ25792 charger and A7672E modem have their own drivers.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "ecotrace_pins.h"

namespace EcoTrace {

/* Configure GPIOs to safe defaults: modem rail OFF, modem PWRKEY idle HIGH,
 * external sensor rail OFF, LED off. Call once at the top of setup(). */
void beginBoard();

/* Start Wire on GPIO6/7 @ 100 kHz. Returns Wire for chaining. */
TwoWire& beginI2C(uint32_t freq_hz = ECO_I2C_FREQ_HZ);

/* Start SPI on the flash pins (CLK18 / MISO5 / MOSI4) with both CS lines high. */
SPIClass& beginSPI();

/* LED helpers */
void ledOn();
void ledOff();
void ledToggle();

/* External sensor rail (GPIO14 high-side switch, active HIGH). Powers sensors on
 * the SENSOR header; the on-board SC7A20 + LTR-303 are on 3V3, not this rail. */
void sensorRail(bool on);

/* Enter deep sleep for `seconds`, holding the modem rail off, PWRKEY high, sensor
 * rail off, and LED off across the sleep so nothing floats. Never returns. */
void deepSleepSeconds(uint32_t seconds);

/* True if this boot came from the deep-sleep timer (vs power-on / reset / brownout). */
bool wokeFromTimer();

}  // namespace EcoTrace
