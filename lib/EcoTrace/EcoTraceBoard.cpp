#include "EcoTraceBoard.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

namespace EcoTrace {

void beginBoard()
{
    /* If we just woke from deepSleepSeconds(), these pins are still latched.
     * Release the holds before reconfiguring them. */
    gpio_hold_dis((gpio_num_t)ECO_PIN_MODEM_PWR_EN);
    gpio_hold_dis((gpio_num_t)ECO_PIN_MODEM_PWRKEY);
    gpio_hold_dis((gpio_num_t)ECO_PIN_SENSOR_PWR);
    gpio_hold_dis((gpio_num_t)ECO_PIN_LED);

    pinMode(ECO_PIN_LED, OUTPUT);
    digitalWrite(ECO_PIN_LED, LOW);

    /* Modem rail OFF, PWRKEY idle HIGH -- do this before anything else so the
     * A7672E is never accidentally powered while we bring the rest up. */
    pinMode(ECO_PIN_MODEM_PWR_EN, OUTPUT);
    digitalWrite(ECO_PIN_MODEM_PWR_EN, LOW);
    pinMode(ECO_PIN_MODEM_PWRKEY, OUTPUT);
    digitalWrite(ECO_PIN_MODEM_PWRKEY, HIGH);

    /* External sensor rail OFF by default. */
    pinMode(ECO_PIN_SENSOR_PWR, OUTPUT);
    digitalWrite(ECO_PIN_SENSOR_PWR, ECO_SENSOR_PWR_OFF);
}

TwoWire& beginI2C(uint32_t freq_hz)
{
    Wire.begin(ECO_PIN_I2C_SDA, ECO_PIN_I2C_SCL, freq_hz);
    return Wire;
}

SPIClass& beginSPI()
{
    SPI.begin(ECO_PIN_SPI_CLK, ECO_PIN_SPI_MISO, ECO_PIN_SPI_MOSI, ECO_PIN_SPI_CS_FLASH);
    pinMode(ECO_PIN_SPI_CS_FLASH, OUTPUT);
    digitalWrite(ECO_PIN_SPI_CS_FLASH, HIGH);
    pinMode(ECO_PIN_SPI_CS_PERIPH, OUTPUT);
    digitalWrite(ECO_PIN_SPI_CS_PERIPH, HIGH);
    return SPI;
}

void ledOn()     { digitalWrite(ECO_PIN_LED, HIGH); }
void ledOff()    { digitalWrite(ECO_PIN_LED, LOW); }
void ledToggle() { digitalWrite(ECO_PIN_LED, !digitalRead(ECO_PIN_LED)); }

void sensorRail(bool on)
{
    digitalWrite(ECO_PIN_SENSOR_PWR, on ? ECO_SENSOR_PWR_ON : ECO_SENSOR_PWR_OFF);
}

void deepSleepSeconds(uint32_t seconds)
{
    /* Assert safe levels, then latch them so they survive deep sleep (single-IO
     * hold persists through deep sleep on the C6). */
    digitalWrite(ECO_PIN_MODEM_PWR_EN, LOW);
    digitalWrite(ECO_PIN_MODEM_PWRKEY, HIGH);
    digitalWrite(ECO_PIN_SENSOR_PWR, ECO_SENSOR_PWR_OFF);
    digitalWrite(ECO_PIN_LED, LOW);

    gpio_hold_en((gpio_num_t)ECO_PIN_MODEM_PWR_EN);
    gpio_hold_en((gpio_num_t)ECO_PIN_MODEM_PWRKEY);
    gpio_hold_en((gpio_num_t)ECO_PIN_SENSOR_PWR);
    gpio_hold_en((gpio_num_t)ECO_PIN_LED);

    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();   /* never returns */
}

bool wokeFromTimer()
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

}  // namespace EcoTrace
