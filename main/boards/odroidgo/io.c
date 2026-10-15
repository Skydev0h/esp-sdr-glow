/* ODROID-GO buttons, battery and status LED. The D-pad is two resistor
 * ladders on ADC1 (Wi-Fi blocks ADC2 on the ESP32, ADC1 is free). */
#include "odroidgo.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define ADC_X ADC_CHANNEL_6   /* GPIO34: left high, right mid */
#define ADC_Y ADC_CHANNEL_7   /* GPIO35: up high, down mid */
#define ADC_BAT ADC_CHANNEL_0 /* GPIO36, 1:2 divider */
#define PIN_LED 2

static const struct { uint8_t pin; bool pullup; unsigned bit; } keys[] = {
    {32, true, OG_A}, {33, true, OG_B}, {13, true, OG_MENU}, {0, false, OG_VOLUME},
    {27, true, OG_SELECT}, {39, false, OG_START},
};
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;

void og_io_init(void) {
    for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        const gpio_config_t in = {
            .pin_bit_mask = 1ULL << keys[i].pin, .mode = GPIO_MODE_INPUT,
            .pull_up_en = keys[i].pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&in));
    }
    const gpio_config_t led = {.pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
    gpio_set_level(PIN_LED, 0);

    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    const adc_oneshot_chan_cfg_t ch = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_X, &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_Y, &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_BAT, &ch));
    const adc_cali_line_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cal, &cali) != ESP_OK) cali = NULL;
}

unsigned og_buttons(void) {
    unsigned held = 0;
    for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
        if (!gpio_get_level(keys[i].pin)) held |= keys[i].bit;
    int x = 0, y = 0;
    adc_oneshot_read(adc, ADC_X, &x);
    adc_oneshot_read(adc, ADC_Y, &y);
    if (x > 3072) held |= OG_LEFT;
    else if (x > 1024) held |= OG_RIGHT;
    if (y > 3072) held |= OG_UP;
    else if (y > 1024) held |= OG_DOWN;
    return held;
}

unsigned og_battery_mv(void) {
    int raw = 0, mv = 0, sum = 0;
    for (int i = 0; i < 16; i++) {
        if (adc_oneshot_read(adc, ADC_BAT, &raw) != ESP_OK) return 0;
        sum += raw;
    }
    raw = (sum + 8) / 16;
    if (cali) adc_cali_raw_to_voltage(cali, raw, &mv);
    else mv = raw * 3300 / 4095;
    return (unsigned)mv * 2;
}

void og_led(bool on) { gpio_set_level(PIN_LED, on); }

uint64_t og_board_pins(void) {
    /* LCD/SD bus, LCD CS/DC/backlight, SD CS, LED, speaker DACs and the
     * output-capable button pins. GPIO 4/12/15 (header) stay free. */
    const unsigned pins[] = {18, 19, 23, 5, 21, 14, 22, 2, 25, 26, 32, 33, 13, 0, 27};
    uint64_t mask = 0;
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) mask |= 1ULL << pins[i];
    return mask;
}
