# API reference - EcoTrace driver library

Every peripheral on the board has a ready-to-call driver in `lib/EcoTrace/`. Include
the header you need and call the functions. This page lists what is available so you
can grab a function without reading the source.

All drivers default to the shared `Wire` / `SPI` instance and the addresses/pins in
`ecotrace_pins.h`, so construction is usually just `BQ25792 bq;`.

> **Logging your own sensor?** The datalogger is a plain starting point - edit
> `readSample()` in `src/datalogger/main.cpp` and read your sensor there, with any
> driver below, an Arduino library, or raw `Wire` / `analogRead`.

Typical setup at the top of `setup()`:

```cpp
#include "EcoTraceBoard.h"
EcoTrace::beginBoard();   // safe defaults: rails off, strapping-safe
EcoTrace::beginI2C();     // Wire on GPIO6/7
EcoTrace::beginSPI();     // SPI on GPIO18/5/4 (only if you use the flash)
```

---

## EcoTraceBoard (`EcoTraceBoard.h`, namespace `EcoTrace`)

| Function | Does |
|----------|------|
| `beginBoard()` | safe GPIO defaults (modem + sensor rails off, releases sleep holds) |
| `beginI2C(freq=100k)` | start `Wire` on GPIO6/7, returns `TwoWire&` |
| `beginSPI()` | start `SPI` on the flash pins, both CS high, returns `SPIClass&` |
| `ledOn() / ledOff() / ledToggle()` | status LED (GPIO1) |
| `sensorRail(bool)` | external sensor rail (GPIO14) on/off |
| `deepSleepSeconds(s)` | hold rails off and deep-sleep; never returns |
| `wokeFromTimer()` | true if this boot was a timer wake (vs power-on/reset) |

## BQ25792 charger / PMIC (`BQ25792.h`)

The important one. Charging, ADC, status, faults, MPPT knobs. Register-accurate to
the TI datasheet.

| Function | Does |
|----------|------|
| `begin() / isPresent()` | detect the chip (part number check) |
| `enableADC(continuous=true) / disableADC()` | turn the 16-bit ADC on/off |
| `enableIbatSensing(on=true)` | needed to read discharge current (IBAT < 0) |
| `readVbat_mV() readVbus_mV() readVsys_mV() readVac1_mV() readVac2_mV()` | voltages (1 mV) |
| `readIbat_mA() readIbus_mA()` | currents, signed (+ = charge / into VBUS) |
| `chargeState() / chargeStateName()` | not-charging / trickle / CC / CV / done ... |
| `vbusPresent() ac1Present() ac2Present() powerGood()` | input presence |
| `readFaults(f0,f1) / hasFault() / faultString(buf,len)` | fault decode |
| `setChargeVoltage_mV() setChargeCurrent_mA() setInputCurrentLimit_mA()` | limits |
| `setVINDPM_mV() / getVINDPM_mV()` | input-voltage regulation point (used for MPPT) |
| `enableCharging(on) setHIZ(on) disableWatchdog()` | control |
| `enableACDRV1(on) enableACDRV2(on)` | gate USB (AC1) / solar (AC2) input FETs |
| `configureCharging(ichg,iindpm,vreg)` | one-call: WD off, HIZ off, set limits, charge on |
| `readReg8/16 writeReg8/16 setBits` | raw register access for experimentation |

Solar note: the BQ25792 has no true MPPT. Track the panel's max-power point by
hill-climbing `setVINDPM_mV()` (see `mpptStep()` in the datalogger).

## ModemA7672 LTE modem (`ModemA7672.h`)

Hand-written AT driver for the A7672E. Use `sendAT()` / `uart()` for anything not
wrapped below.

| Function | Does |
|----------|------|
| `begin(baud=115200)` | power rail + PWRKEY sequence, wait for AT/OK |
| `powerOff()` | graceful AT+CPOF then drop the rail |
| `statusHigh()` | modem STATUS pin (powered?) |
| `sendAT(cmd,expect="OK",timeout) / waitFor(needle,timeout)` | raw AT |
| `lastResponse() / uart()` | last reply buffer / the underlying Stream |
| `getInfo(buf,len)` | ATI |
| `simReady()` | AT+CPIN? -> READY |
| `waitForNetwork(timeout)` | registration (home/roaming) |
| `signalQuality_dBm()` | AT+CSQ in dBm |
| `batteryVoltage_mV()` | modem's own VBAT sense (AT+CBC) |
| `connectGPRS(apn)` | CGDCONT + CGACT |
| `httpGet(url,status[,body,len])` | HTTP GET, returns true on 2xx |
| `httpPost(url,ctype,payload,status[,body,len])` | HTTP POST, returns true on 2xx |
| `getUnixTimeMs()` | network time (AT+CCLK) as unix ms |
| `gpsEnable() / gpsGetFix(...)` | 🚧 stub - GPS untested on Rev A hardware |

## LTR303 ambient light (`LTR303.h`)

| Function | Does |
|----------|------|
| `begin() / isPresent()` | detect + activate |
| `read(ch0,ch1)` | CH0 = visible+IR, CH1 = IR |
| `readCh0()` | convenience |
| `readLux()` | rough lux estimate (trends, not photometry) |
| `setActive(bool) / powerDown()` | active / standby |

## SC7A20 accelerometer (`SC7A20.h`)

| Function | Does |
|----------|------|
| `begin() / isPresent()` | auto-detect 0x18/0x19, WHO_AM_I, 100 Hz |
| `readMilliG(x,y,z)` | acceleration in mg (+/-2 g) |
| `powerDown()` | low-power |
| `address()` | detected I2C address |

## ExtFlash - GD25Q256 NOR flash (`ExtFlash.h`)

Persists across power loss. Erase-before-write; page = 256 B, sector = 4 KB. Low
16 MB via 3-byte addressing (extend for the upper half).

| Function | Does |
|----------|------|
| `begin()` | wake + confirm GigaDevice JEDEC |
| `jedecId()` | mfg/type/capacity |
| `read(addr,buf,n)` | read bytes |
| `eraseSector(addr)` | erase 4 KB to 0xFF |
| `writePage(addr,buf,n)` | program <=256 B within one page |
| `powerDown() / wake()` | deep power-down for sleep |

## ATECC608B secure element (`ATECC608B.h`)

Thin: presence + wake/sleep only. For provisioning / ECDSA signing use Microchip
CryptoAuthLib or SparkFun_ATECCX08a.

| Function | Does |
|----------|------|
| `begin()` | wake pulse + probe (0x60 or 0x35) |
| `wake() / sleep()` | power state |
| `address()` | detected I2C address |
