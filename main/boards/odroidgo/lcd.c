/* ODROID-GO ILI9341 (320x240, landscape) on SPI2 at 40 MHz. Only panel IO is
 * needed: init list, window, RAMWR. Init values from Retro-Go's odroid-go
 * target. The SD card shares the bus; its CS (GPIO22) is held high. */
#include "odroidgo.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_CS 5
#define PIN_DC 21
#define PIN_BL 14
#define PIN_SD_CS 22

static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t free_strips;
static uint16_t *strips[2];
static unsigned next_strip;

static bool on_done(esp_lcd_panel_io_handle_t h, esp_lcd_panel_io_event_data_t *e, void *ctx) {
    (void)h; (void)e; (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(free_strips, &woken);
    return woken == pdTRUE;
}

static void cmd(uint8_t c, const uint8_t *data, size_t n) { esp_lcd_panel_io_tx_param(io, c, data, n); }

void og_lcd_init(void) {
    free_strips = xSemaphoreCreateCounting(2, 2);
    for (int i = 0; i < 2; i++) {
        strips[i] = heap_caps_malloc(OG_LCD_W * OG_STRIP_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(strips[i]);
    }
    const gpio_config_t sd = {.pin_bit_mask = 1ULL << PIN_SD_CS, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&sd));
    gpio_set_level(PIN_SD_CS, 1);

    /* Backlight PWM, dark until the first frame is drawn. */
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const ledc_channel_config_t bl = {
        .gpio_num = PIN_BL, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bl));

    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = OG_LCD_W * OG_STRIP_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = PIN_DC, .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000, .trans_queue_depth = 4,
        .on_color_trans_done = on_done, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    static const struct { uint8_t cmd, n, data[15]; uint16_t delay_ms; } init[] = {
        {0x01, 0, {0}, 5},                                     /* SWRESET */
        {0x3A, 1, {0x55}, 0},                                  /* COLMOD: 16 bit */
        {0xCF, 3, {0x00, 0xC3, 0x30}, 0},
        {0xED, 4, {0x64, 0x03, 0x12, 0x81}, 0},
        {0xE8, 3, {0x85, 0x00, 0x78}, 0},
        {0xCB, 5, {0x39, 0x2C, 0x00, 0x34, 0x02}, 0},
        {0xF7, 1, {0x20}, 0},
        {0xEA, 2, {0x00, 0x00}, 0},
        {0xC0, 1, {0x1B}, 0},                                  /* Power control */
        {0xC1, 1, {0x12}, 0},
        {0xC5, 2, {0x32, 0x3C}, 0},                            /* VCM control */
        {0xC7, 1, {0x91}, 0},
        {0x36, 1, {0xA8}, 0},                                  /* MADCTL: MY|MV|BGR, landscape */
        {0xB1, 2, {0x00, 0x10}, 0},                            /* Frame rate 119 Hz */
        {0xB6, 2, {0x0A, 0xA2}, 0},                            /* Display function control */
        {0xF6, 2, {0x01, 0x30}, 0},
        {0xF2, 1, {0x00}, 0},                                  /* 3-gamma off */
        {0x26, 1, {0x01}, 0},                                  /* Gamma curve */
        {0xE0, 15, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 0},
        {0xE1, 15, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 0},
        {0x11, 0, {0}, 10},                                    /* SLPOUT */
    };
    for (unsigned i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
        cmd(init[i].cmd, init[i].n ? init[i].data : NULL, init[i].n);
        if (init[i].delay_ms) vTaskDelay(pdMS_TO_TICKS(init[i].delay_ms));
    }
    /* Clear before the display and backlight come on so no noise flashes. */
    for (int y = 0; y < OG_LCD_H; y += OG_STRIP_H) {
        uint16_t *s = og_lcd_strip();
        for (unsigned i = 0; i < OG_LCD_W * OG_STRIP_H; i++) s[i] = 0;
        og_lcd_blit(0, y, OG_LCD_W, OG_STRIP_H, s);
    }
    cmd(0x29, NULL, 0); /* DISPON */
}

void og_lcd_backlight(unsigned percent) {
    if (percent > 100) percent = 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, percent * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

uint16_t *og_lcd_strip(void) {
    xSemaphoreTake(free_strips, pdMS_TO_TICKS(200));
    uint16_t *s = strips[next_strip];
    next_strip ^= 1;
    return s;
}

void og_lcd_idle(void) {
    for (int i = 0; i < 2; i++) xSemaphoreTake(free_strips, pdMS_TO_TICKS(100));
    for (int i = 0; i < 2; i++) xSemaphoreGive(free_strips);
}

void og_lcd_blit(int x, int y, int w, int h, uint16_t *strip) {
    if (w <= 0 || h <= 0) {
        xSemaphoreGive(free_strips);
        return;
    }
    const int x1 = x + w - 1, y1 = y + h - 1;
    const uint8_t ca[4] = {(uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(x1 >> 8), (uint8_t)x1};
    const uint8_t ra[4] = {(uint8_t)(y >> 8), (uint8_t)y, (uint8_t)(y1 >> 8), (uint8_t)y1};
    cmd(0x2A, ca, 4); /* CASET */
    cmd(0x2B, ra, 4); /* RASET */
    esp_lcd_panel_io_tx_color(io, 0x2C, strip, (size_t)w * h * 2);
}
