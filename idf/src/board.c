#include "board.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

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
