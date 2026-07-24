/*
 * ATECC608B.h  --  thin driver for the ATECC608B secure element (I2C 0x60 / 0x35).
 * Presence + wake/sleep only. Full crypto (provisioning, ECDSA sign/verify) is out
 * of scope here -- use Microchip CryptoAuthLib or SparkFun_ATECCX08a for that; this
 * class just gets the chip awake and confirms it is on the bus.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ecotrace_pins.h"

class ATECC608B {
public:
    explicit ATECC608B(TwoWire& bus = Wire) : _bus(&bus), _addr(0) {}

    /* Send a wake pulse and probe. Returns true if the chip ACKs (sets address()). */
    bool begin();
    uint8_t address() const { return _addr; }

    void wake();    /* start bit on 0x00 holds SDA low long enough to wake the chip */
    void sleep();   /* write the Sleep word address (0x01) */

private:
    TwoWire* _bus;
    uint8_t  _addr;
    bool probe(uint8_t a);
};
