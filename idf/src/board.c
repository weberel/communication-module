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
    out(ECO_PIN_SENSOR_PWR, 0);
    out(ECO_PIN_LED, 0);

    gpio_reset_pin(ECO_PIN_QON);
    gpio_set_direction(ECO_PIN_QON, GPIO_MODE_INPUT);   /* external pull-up */
}

void board_deep_sleep(uint32_t seconds)
{
    gpio_set_level(ECO_PIN_MODEM_PWR_EN, 0);
    gpio_set_level(ECO_PIN_MODEM_PWRKEY, 0);   /* deasserted: no inverter current */
    gpio_set_level(ECO_PIN_SENSOR_PWR, 0);
    gpio_set_level(ECO_PIN_LED, 0);

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
