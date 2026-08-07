/*
 * button_test -- QON button + BQ25792 ship mode, nothing else.
 *
 * Never sleeps, so the USB port is always there and flashing never has to race
 * a wake window. LED is ON whenever this firmware runs: if the board goes dark
 * and stays dark, it is genuinely powered off.
 *
 *   pio run -e button_test -t upload && pio device monitor
 *
 * Short press  -> prints the press and its duration.
 * Hold >= 3 s  -> LED flickers (armed), then on RELEASE the BQ is commanded
 *                 into ship mode. On battery the board must die instantly.
 *                 If it survives, the register readback says why.
 * While off    -> hold the button ~1 s: the BQ leaves ship mode and the board
 *                 boots again (LED comes back on).
 */
#include <Arduino.h>
#include "EcoTraceBoard.h"
#include "BQ25792.h"

#define SHIP_HOLD_MS 3000

BQ25792 bq;

/* Latched result of the last ship-mode attempt. Reprinted every 2 s so a
 * battery-only attempt can be read back after USB is reconnected. */
static char s_last_attempt[700] = "none yet";

/* Try every SDRV_CTRL variant in one go and latch the full evidence, so a
 * battery-only run needs no serial connection while it happens. */
static void shipSweep(void)
{
    struct { uint8_t code; uint8_t dly; const char* name; } v[] = {
        { 0x02, 1, "ship(10) dly=1" },
        { 0x02, 0, "ship(10) dly=0" },
        { 0x01, 1, "shutdown(01)"   },
        { 0x03, 1, "pwr-reset(11)"  },
    };
    size_t n = 0;
    bq.setShipFETPresent(true);   /* required or every SDRV_CTRL write is dropped */
    /* If ship mode really executes, the ship FET opens and the system can only
     * be fed through body diodes -- VSYS must sag well below VBAT. If VSYS
     * keeps tracking VBAT within a few mV, the FET never opened. */
    bq.enableADC();
    uint16_t vbat0 = bq.readVbat_mV(), vsys0 = bq.readVsys_mV();
    uint8_t r1b = bq.readReg8(0x1B), r1c = bq.readReg8(0x1C);
    n += snprintf(s_last_attempt + n, sizeof(s_last_attempt) - n,
                  "t=%lus REG1B=0x%02X(VBUS_PRES=%u) REG1C=0x%02X(VBUS_STAT=%u) QON=%d |",
                  (unsigned long)(millis() / 1000), r1b, r1b & 1, r1c,
                  (r1c >> 1) & 0x0F, digitalRead(ECO_PIN_QON));

    for (size_t i = 0; i < sizeof(v) / sizeof(v[0]) && n < sizeof(s_last_attempt) - 90; i++) {
        bq.disableWatchdog();
        uint8_t wd   = bq.readReg8(0x10) & 0x07;
        uint8_t orig = bq.readReg8(0x11);
        bool ok = bq.writeReg8(0x11, (uint8_t)((orig & ~0x07) | (v[i].code << 1) | v[i].dly));
        uint8_t a = bq.readReg8(0x11);
        delay(100);
        uint8_t b = bq.readReg8(0x11);
        delay(900);
        uint8_t c = bq.readReg8(0x11);
        uint16_t vbat1 = bq.readVbat_mV(), vsys1 = bq.readVsys_mV();
        n += snprintf(s_last_attempt + n, sizeof(s_last_attempt) - n,
                      " [%s wd=%u w=%d %02X->%02X,%02X,%02X VSYS %u->%u VBAT %u->%u]",
                      v[i].name, wd, ok, orig, a, b, c, vsys0, vsys1, vbat0, vbat1);
    }
}

static void dumpRegs(const char* tag)
{
    uint8_t r10 = bq.readReg8(0x10), r11 = bq.readReg8(0x11);
    uint8_t r12 = bq.readReg8(0x12), r13 = bq.readReg8(0x13);
    uint8_t r1b = bq.readReg8(0x1B), r1c = bq.readReg8(0x1C);
    uint8_t r1d = bq.readReg8(0x1D), r1e = bq.readReg8(0x1E);
    Serial.printf("%s: REG10=0x%02X(WD=%u) REG11=0x%02X(SDRV=%u DLY=%u) REG12=0x%02X REG13=0x%02X | "
                  "REG1B=0x%02X(VBUS_PRES=%u) REG1C=0x%02X REG1D=0x%02X REG1E=0x%02X QON=%d\n",
                  tag, r10, r10 & 0x07, r11, (r11 >> 1) & 3, r11 & 1, r12, r13,
                  r1b, r1b & 1, r1c, r1d, r1e, digitalRead(ECO_PIN_QON));
}

static void report(const char* tag)
{
    uint8_t r11 = bq.readReg8(0x11);
    Serial.printf("%s: REG11=0x%02X SDRV_CTRL=%u VBUS=%s VBAT=%u mV\n",
                  tag, r11, (r11 >> 1) & 0x03,
                  bq.vbusPresent() ? "present" : "absent",
                  bq.readVbat_mV());
}

void setup()
{
    Serial.begin(115200);
    delay(1500);
    EcoTrace::beginBoard();
    EcoTrace::beginI2C();
    EcoTrace::ledOn();          /* LED on = this firmware is running */

    pinMode(ECO_PIN_QON, INPUT);

    Serial.println("\n=== button_test ===");
    Serial.printf("BQ25792 %s\n", bq.begin() ? "found" : "NOT FOUND");
    bq.enableADC();
    report("boot");
    Serial.println("press the button (hold 3 s to power off)");
}

void loop()
{
    /* 1 Hz heartbeat: unmistakable "this board is powered and running".
     * On the battery-only ship test, the heartbeat stopping IS the result. */
    static uint32_t beat = 0;
    if (millis() - beat >= 500) {
        beat = millis();
        EcoTrace::ledToggle();
    }
    static uint32_t status_at = 0;
    if (millis() - status_at >= 2000) {
        status_at = millis();
        Serial.printf("status: REG11=0x%02X VBUS=%s VBAT=%u mV | last ship attempt: %s\n",
                      bq.readReg8(0x11), bq.vbusPresent() ? "present" : "absent",
                      bq.readVbat_mV(), s_last_attempt);
    }

    /* Serial commands so experiments don't need a human at the button:
     *   d = dump registers      w = disable watchdog + dump
     *   s = ship mode attempt   r = system power reset (should reboot us)
     *   h = shutdown mode attempt                                        */
    if (Serial.available()) {
        int c = Serial.read();
        if (c == 'd') dumpRegs("dump");
        else if (c == 'w') { bq.disableWatchdog(); dumpRegs("after WD off"); }
        else if (c == 's') {
            dumpRegs("before ship");
            bq.enterShipMode();
            delay(50);  dumpRegs("t+50ms");
            delay(1500); dumpRegs("t+1.5s");
        } else if (c == 'r') {
            Serial.println("system power reset (expect a reboot)...");
            Serial.flush();
            bq.systemPowerReset();
            delay(2000);
            dumpRegs("survived power reset");
        } else if (c == 'p') {
            /* SFET_PRESENT (REG14 bit 7): tell the charger an external ship
             * FET exists. Without it SDRV_CTRL is ignored. */
            uint8_t r14 = bq.readReg8(0x14);
            bq.writeReg8(0x14, r14 | 0x80);
            uint8_t r14b = bq.readReg8(0x14);
            bq.disableWatchdog();
            uint8_t o = bq.readReg8(0x11);
            bq.writeReg8(0x11, (uint8_t)((o & ~0x07) | 0x06 | 0x01));   /* pwr reset */
            uint8_t a = bq.readReg8(0x11);
            delay(400);
            uint8_t b = bq.readReg8(0x11);
            Serial.printf("SFET_PRESENT: REG14 %02X->%02X (bit7=%u) | SDRV=11: %02X->%02X,%02X %s\n",
                          r14, r14b, (r14b >> 7) & 1, o, a, b,
                          (((a >> 1) & 3) || ((b >> 1) & 3)) ? "*** HELD ***" : "still cleared");
        } else if (c == 'x') {
            /* Systematically try preconditions that might gate SDRV actions.
             * Target = system power reset (11), which per the datasheet works
             * regardless of adapter, so it is testable over USB. Success =
             * SDRV_CTRL holds non-zero, or the board reboots. */
            for (int k = 0; k < 5; k++) {
                bq.disableWatchdog();
                const char* name = "";
                switch (k) {
                case 0: name = "REG_RST first";
                    bq.writeReg8(0x09, bq.readReg8(0x09) | 0x40); delay(50);
                    bq.disableWatchdog(); break;
                case 1: name = "write twice"; break;
                case 2: name = "EN_HIZ=1 first";
                    bq.writeReg8(0x0F, bq.readReg8(0x0F) | 0x04); break;
                case 3: name = "EN_CHG=0 first";
                    bq.writeReg8(0x0F, bq.readReg8(0x0F) & ~0x20); break;
                case 4: name = "EN_EXT_ILIM=0 first";
                    bq.writeReg8(0x14, bq.readReg8(0x14) & ~0x02); break;
                }
                uint8_t o = bq.readReg8(0x11);
                uint8_t val = (uint8_t)((o & ~0x07) | 0x06 | 0x01);
                bq.writeReg8(0x11, val);
                if (k == 1) bq.writeReg8(0x11, val);      /* the double write */
                uint8_t a = bq.readReg8(0x11);
                delay(400);
                uint8_t b = bq.readReg8(0x11);
                Serial.printf("[%-20s] %02X -> %02X,%02X %s\n", name, o, a, b,
                              (((a >> 1) & 3) || ((b >> 1) & 3)) ? "*** HELD ***" : "");
                /* undo */
                bq.writeReg8(0x0F, (bq.readReg8(0x0F) | 0x20) & ~0x04);
            }
            Serial.println("precondition sweep done");
        } else if (c == 'z') {
            /* Hypothesis: a running ADC blocks SDRV actions (ship mode slows
             * the charger clock). Turn the ADC off first, then command a
             * system power reset - which per the datasheet works even with an
             * adapter present, so it is testable over USB. */
            bq.disableADC();
            bq.disableWatchdog();
            delay(50);
            uint8_t orig = bq.readReg8(0x11);
            bool w = bq.writeReg8(0x11, (uint8_t)((orig & ~0x07) | 0x06 | 0x01));
            uint8_t a = bq.readReg8(0x11);
            Serial.printf("ADC off + SDRV=11: REG2E=0x%02X w=%d %02X->%02X\n",
                          bq.readReg8(0x2E), w, orig, a);
            delay(1500);
            Serial.printf("  after 1.5s REG11=0x%02X (a reboot would have cut this line)\n",
                          bq.readReg8(0x11));
        } else if (c == 'y') {
            /* SDRV_CTRL=11 (system power reset) with SDRV_DLY=0, i.e. the
             * 10 s delayed path. Poll for 15 s: a reboot or a register that
             * finally holds tells us the delay path is the working one. */
            uint8_t orig = bq.readReg8(0x11);
            bq.disableWatchdog();
            bool w = bq.writeReg8(0x11, (uint8_t)((orig & ~0x07) | 0x06));  /* DLY=0 */
            Serial.printf("SDRV=11 DLY=0 write_ok=%d; polling 15 s...\n", w);
            for (int i = 0; i < 15; i++) {
                delay(1000);
                Serial.printf("  t+%2ds REG11=0x%02X\n", i + 1, bq.readReg8(0x11));
            }
        } else if (c == 't') {
            /* Does REG11 accept ANY bit? Toggle EN_9V (bit 4), then try
             * SDRV_CTRL and sample it as fast as I2C allows. */
            uint8_t orig = bq.readReg8(0x11);
            bool w1 = bq.writeReg8(0x11, orig | 0x10);
            uint8_t rb1 = bq.readReg8(0x11);
            bq.writeReg8(0x11, orig);
            Serial.printf("EN_9V test: orig=0x%02X write_ok=%d readback=0x%02X -> bit4 %s\n",
                          orig, w1, rb1, (rb1 & 0x10) ? "STUCK (writes work)" : "did not stick");
            bool w2 = bq.writeReg8(0x11, (uint8_t)((orig & ~0x07) | 0x06 | 0x01));
            uint8_t s0 = bq.readReg8(0x11), s1 = bq.readReg8(0x11), s2 = bq.readReg8(0x11);
            Serial.printf("SDRV=11 test: write_ok=%d reads=0x%02X,0x%02X,0x%02X\n", w2, s0, s1, s2);
        } else if (c == 'h') {
            Serial.println("shutdown mode attempt...");
            bq.disableWatchdog();
            uint8_t v = bq.readReg8(0x11);
            bq.writeReg8(0x11, (uint8_t)((v & ~0x07) | (0x01 << 1) | 0x01));
            delay(50);  dumpRegs("t+50ms");
            delay(1500); dumpRegs("t+1.5s");
        }
    }

    if (digitalRead(ECO_PIN_QON) == HIGH) { delay(10); return; }
    delay(50);                                        /* debounce */
    if (digitalRead(ECO_PIN_QON) == HIGH) return;     /* bounce, not a press */

    uint32_t t0 = millis();
    bool armed = false;
    while (digitalRead(ECO_PIN_QON) == LOW) {
        uint32_t held = millis() - t0;
        if (!armed && held >= SHIP_HOLD_MS) {
            armed = true;
            Serial.println("armed: release to power off");
            for (int i = 0; i < 3; i++) {          /* visible "armed" flicker */
                EcoTrace::ledOff(); delay(80);
                EcoTrace::ledOn();  delay(80);
            }
        }
        delay(10);
    }
    uint32_t held = millis() - t0;

    if (!armed) {
        Serial.printf("short press (%lu ms)\n", (unsigned long)held);
        delay(50);   /* release bounce */
        return;
    }

    /* Released. QON is high again, so the BQ will not read our ship command as
     * its own wake signal. */
    Serial.printf("released after %lu ms -> entering ship mode\n", (unsigned long)held);
    Serial.flush();
    delay(150);

    bq.enterShipMode();          /* sets SFET_PRESENT + disables watchdog */
    delay(1500);
    /* Only reached if the board did not power off: collect the evidence. */
    shipSweep();
    Serial.printf("still alive: %s\n", s_last_attempt);
}
