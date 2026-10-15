/* ODROID-GO (Hardkernel, ESP32-D0WD WROVER, 2018): on-device receiver UI on
 * the 2.4" ILI9341 display, D-pad and buttons, and the DAC-driven speaker.
 * Pin map follows Retro-Go's odroid-go target. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "sdr_local.h"
#include "sdr_view.h"

/* ILI9341 320x240 landscape on SPI2 (shared with the SD card, CS 22). */
#define OG_LCD_W 320
#define OG_LCD_H 240
#define OG_STRIP_H 16
void og_lcd_init(void);
void og_lcd_backlight(unsigned percent);
/* A free OG_LCD_W * OG_STRIP_H DMA strip (two alternate); waits while both
 * are still being sent. */
uint16_t *og_lcd_strip(void);
/* Queues w*h big-endian RGB565 pixels from a strip; the strip is busy until
 * the transfer ends, so take the next one with og_lcd_strip(). */
void og_lcd_blit(int x, int y, int w, int h, uint16_t *strip);
/* Waits until no strip is being sent. Call before every RF capture: SPI
 * traffic to the panel during a capture shows up as a comb of spurs. */
void og_lcd_idle(void);

/* Buttons: D-pad on ADC1 (GPIO34/35), the rest active-low GPIOs. */
enum {
    OG_UP = 1u << 0, OG_DOWN = 1u << 1, OG_LEFT = 1u << 2, OG_RIGHT = 1u << 3,
    OG_A = 1u << 4, OG_B = 1u << 5, OG_MENU = 1u << 6, OG_VOLUME = 1u << 7,
    OG_SELECT = 1u << 8, OG_START = 1u << 9,
};
void og_io_init(void);
unsigned og_buttons(void);
unsigned og_battery_mv(void); /* 0 when unavailable; 16 readings averaged */
void og_led(bool on);         /* blue status LED, GPIO2 */
/* Output pins the board drivers own (reserved from GPIO discovery). */
uint64_t og_board_pins(void);

/* Cosine generator on both DAC channels in antiphase (GPIO25/26 drive the
 * speaker amplifier differentially). Volume 0 (mute) .. OG_TONE_VOLUMES-1. */
#define OG_TONE_VOLUMES 5
void og_tone_init(void);
void og_tone(unsigned hz); /* 0 = silence */
unsigned og_tone_hz(void); /* what is playing, 0 = silence */
/* A Geiger-counter click: one short biphasic pulse written straight to the
 * DACs (0.3 ms), loudness 0..1 spanning about 26 dB; ignored while a tone
 * plays or when muted. */
void og_click(float loudness);
void og_tone_volume(unsigned level);

/* Burst-aware band power for one capture (zb.c). Each band is a centre
 * frequency in MHz with a common half width; segments of ZB_FFT_N samples are
 * Hann-windowed and transformed, so short packets inside a long capture show
 * up in the per-band peak even when the capture average hides them. */
#define ZB_FFT_N 256
#define ZB_MAX_BANDS 16
typedef struct {
    float peak[ZB_MAX_BANDS]; /* max over time (4-segment smoothing), dB per bin */
    float mean[ZB_MAX_BANDS]; /* mean over the capture, dB per bin */
    float busy[ZB_MAX_BANDS]; /* share of smoothed windows above busy_db (airtime) */
    float floor;              /* median bin of the capture-average spectrum, dB */
    bool clip;                /* ADC near full scale in more than 0.1 % of samples */
    int peak_abs;             /* largest |I| or |Q|, of 511 */
} zb_result_t;
bool zb_init(void);
/* Bands outside the usable part of the span are reported as -200 dB.
 * busy_db is an absolute threshold on the same scale as peak[]; below -150
 * no airtime is computed. */
bool zb_analyze(const uint32_t *iq, unsigned n, unsigned msps, unsigned lo_mhz,
                const float *band_mhz, int bands, float half_mhz, float busy_db,
                zb_result_t *out);
