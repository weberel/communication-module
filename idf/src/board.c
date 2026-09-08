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

/* Whether the shared sensor INT participates in the next deep sleep. */
static bool s_motion_wake;

void board_init(void)
{
    /* Release holds from a previous deep sleep before reconfiguring. */
    gpio_hold_dis(ECO_PIN_MODEM_PWR_EN);
    gpio_hold_dis(ECO_PIN_MODEM_PWRKEY);
    gpio_hold_dis(ECO_PIN_SENSOR_PWR);
    gpio_hold_dis(ECO_PIN_LED);
    gpio_hold_dis(ECO_PIN_MODEM_TX);
    gpio_hold_dis(ECO_PIN_MODEM_RX);

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
    gpio_reset_pin(ECO_PIN_INT_SHARED);
    gpio_set_direction(ECO_PIN_INT_SHARED, GPIO_MODE_INPUT);   /* external pull-up */
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
     * gpio_reset_pin() turns the pull-up ON ("for powersave reasons, the GPIO
     * should not be floating, select pullup") and gpio_set_direction() never
     * clears it, so out() leaves each pad driving a level WITH its ~42k internal
     * pull-up still enabled. A pad driven LOW then fights its own output driver
     * for the whole sleep: 3.3 V / 42k = 78 uA per pad. That pull-up lives in
     * the always-on domain, so an UNHELD pad powers down and the conflict
     * disappears -- which is exactly why the cost only appears once
     * gpio_hold_en() latches the state, and why it looked like "holding is
     * expensive" rather than "we are holding a pull-up into a driver".
     *
     * Measured 2026-09-08 (JS220, comm board alone, USB out, battery lead):
     *   no holds                       128.5 uA
     *   + hold LED (driven LOW)        206.9 uA   (+78.4)
     *   + hold PWR_EN+PWRKEY (LOW)     284.6 uA   (+156.1, i.e. 78 each)
     *   + hold SENSOR_PWR (driven HIGH)345.3 uA   (+0 pull-up; the rest is the
     *                                              energised rail itself)
     * A pad held HIGH costs nothing, because driver and pull-up agree. */
    gpio_pullup_dis(ECO_PIN_MODEM_PWR_EN);
    gpio_pullup_dis(ECO_PIN_MODEM_PWRKEY);
    gpio_pullup_dis(ECO_PIN_LED);

    gpio_hold_en(ECO_PIN_MODEM_PWR_EN);
    gpio_hold_en(ECO_PIN_MODEM_PWRKEY);
    gpio_hold_en(ECO_PIN_SENSOR_PWR);   /* keeps the ultrasonic board powered */
    gpio_hold_en(ECO_PIN_LED);
    /* NOTE 2026-09-08, corrected the same day.
     *
     * The holds above DO survive deep sleep on the ESP32-C6, including on
     * GPIO14/GPIO23, which are not RTC pins. esp32c6/soc_caps.h defines BOTH
     * SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP and SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP,
     * and driver/gpio.h guards the bulk API with
     *     #if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
     * so gpio_deep_sleep_hold_en() is deliberately compiled OUT here: the C6
     * holds pads individually and gpio_hold_en() is all that is needed. Trying
     * to call it fails to link, which is what produced the earlier, wrong claim
     * that these holds "do not survive the sleep at all". Being an RTC/LP pin
     * (GPIO0..7) governs EXT1 wake and LP-core access, not hold.
     *
     * Second correction: the back-feed with the rail DOWN measures 60-72 uA,
     * not 1.4 mA (2026-09-08: comm alone 128.3 uA, vs 188.9-199.9 uA with the
     * ultrasonic and pressure boards attached, measured on a staircase build
     * that bypasses this function and therefore arms no hold). The 1.4 mA
     * figure belongs to board_sensor_power_cycle(), which deliberately drives
     * SDA/SCL LOW through the 4k7 pull-ups (2 x 3.3 V / 4k7 = 1.404 mA) for the
     * length of the off window only. */


    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    /* Both sources are active LOW (button pulls QON down; the SC7A20's INT is
     * configured H_LACTIVE), so they share one ANY_LOW mask. */
    uint64_t mask = 1ULL << ECO_PIN_QON;
    if (s_motion_wake) mask |= 1ULL << ECO_PIN_INT_SHARED;
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

bool board_woke_from_timer(void)
{
    return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

/* EXT1 carries two pins, so the cause alone is not enough -- ask which one. */
bool board_woke_from_button(void)
{
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT1) return false;
    return (esp_sleep_get_ext1_wakeup_status() & (1ULL << ECO_PIN_QON)) != 0;
}

bool board_woke_from_motion(void)
{
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT1) return false;
    return (esp_sleep_get_ext1_wakeup_status() & (1ULL << ECO_PIN_INT_SHARED)) != 0;
}

void board_set_motion_wake(bool enable) { s_motion_wake = enable; }


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
