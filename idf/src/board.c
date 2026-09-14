#include "board.h"
#include "i2c_bus.h"

#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 

static void out(gpio_num_t pin, int level)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, level);
}

void board_init(void)
{
    /* Release holds from a previous deep sleep before reconfiguring. */
    gpio_hold_dis(ECO_PIN_MODEM_PWR_EN);
    gpio_hold_dis(ECO_PIN_MODEM_PWRKEY);
    gpio_hold_dis(ECO_PIN_SENSOR_PWR);
    gpio_hold_dis(ECO_PIN_LED);

    out(ECO_PIN_MODEM_PWR_EN, 0);   /* modem rail OFF first, always */
    out(ECO_PIN_MODEM_PWRKEY, 1);   /* idle-high while awake */
    /* HIGH, not low. The sensor rail is always-on policy now (only
     * board_sensor_power_cycle() drops it), and driving it low here glitched
     * the ultrasonic module on every comm-module reboot -- costing it a 3 s
     * reboot and clearing its autonomous mode. Coming out of deep sleep the
     * pin was already held high, so this keeps it glitch-free too. */
    out(ECO_PIN_SENSOR_PWR, 1);
    out(ECO_PIN_LED, 0);

    gpio_reset_pin(ECO_PIN_QON);
    gpio_set_direction(ECO_PIN_QON, GPIO_MODE_INPUT);   /* external pull-up */
}

void board_deep_sleep(uint32_t seconds)
{
    gpio_set_level(ECO_PIN_MODEM_PWR_EN, 0);
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 0);   /* deasserted: no inverter current */
    /* SENSOR rail stays ON through deep sleep, deliberately.
     *
     * Dropping it does not save power, it costs power: with the rail down, our
     * always-on 4k7 bus pull-ups feed the sleeping board through its ESD clamps
     * (~1.4 mA continuous, for the entire 5-minute sleep) and clamp SDA/SCL at
     * the same time. Leaving it up removes that path, and the ultrasonic board
     * gates its own AFE rails between measurements anyway, so its idle draw is
     * the MSP430 in LPM3 plus an LDO -- microamps.
     *
     * The rail is only ever dropped by board_sensor_power_cycle(), i.e. when
     * the module needs a reboot. */
    gpio_set_level(ECO_PIN_SENSOR_PWR, 1);
    gpio_set_level(ECO_PIN_LED, 0);

    /* Drop the internal pull-up on every pad held LOW, THEN hold.
     *
     * gpio_reset_pin() turns the pull-up ON ("the GPIO should not be floating,
     * select pullup") and gpio_set_direction() never clears it, so out() leaves
     * each pad driving a level WITH its ~42k internal pull-up still enabled. A
     * pad driven LOW then fights its own output driver for the whole sleep:
     * 3.3 V / 42k = 78 uA per pad. The pull-up is in the always-on domain, so an
     * UNHELD pad powers down and the conflict disappears -- which is why the
     * cost only shows once gpio_hold_en() latches the state, and why it looked
     * like "holding is expensive" rather than "we are holding a pull-up into a
     * driver".
     *
     * Measured 2026-09-08 (JS220, battery lead, docs/power-2026-09-08.md):
     *   shipping firmware        1467.5 uA
     *   after this fix            356.5 uA
     * A pad held HIGH costs nothing -- driver and pull-up agree -- so
     * SENSOR_PWR is deliberately left alone.
     *
     * BACKPORTED ALONE, 2026-09-13, onto the 95d50a7 build that ran the clean
     * three weeks. It is the ONLY change from 09-05..09-08 with a measured
     * benefit: the same report clears esp_pm_configure() at +0.1 uA and motion
     * wake at 0.4 uA, i.e. nothing. This fix touches no clock, no rail and no
     * I2C timing, which is why it is the one worth carrying forward. */
    gpio_pullup_dis(ECO_PIN_MODEM_PWR_EN);
    gpio_pullup_dis(ECO_PIN_MODEM_PWRKEY);
    gpio_pullup_dis(ECO_PIN_LED);

    gpio_hold_en(ECO_PIN_MODEM_PWR_EN);
    gpio_hold_en(ECO_PIN_MODEM_PWRKEY);
    gpio_hold_en(ECO_PIN_SENSOR_PWR);
    gpio_hold_en(ECO_PIN_LED);

    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_sleep_enable_ext1_wakeup(1ULL << ECO_PIN_QON, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

bool board_woke_from_timer(void)
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

bool board_woke_from_button(void)
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
}


void board_sensor_power(bool on)
{
    gpio_set_level(ECO_PIN_SENSOR_PWR, on ? 1 : 0);
}

void board_sensor_power_cycle(uint32_t off_ms)
{
    /* Order matters. Holding the bus low FIRST is what makes the rail actually
     * fall: an idle-high bus back-feeds the ultrasonic board through its ESD
     * clamps and parks it too low to run but too high to reset. Measured
     * 2026-08-15; see docs/I2C_LINK.md 2.0. */
    ESP_LOGI("board", "sensor power cycle (%lu ms)", (unsigned long) off_ms);
    eco_i2c_hold_low();
    gpio_set_level(ECO_PIN_SENSOR_PWR, 0);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
    gpio_set_level(ECO_PIN_SENSOR_PWR, 1);
    vTaskDelay(pdMS_TO_TICKS(BOARD_SENSOR_BOOT_MS));
    eco_i2c_release();
}
