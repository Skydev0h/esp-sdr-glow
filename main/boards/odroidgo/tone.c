/* Speaker tone from the DAC cosine generator: no CPU, no I2S. GPIO25 and
 * GPIO26 feed the amplifier differentially (Retro-Go drives them as a
 * differential pair too), so the channels run 180 degrees apart. Pitch and
 * level change in place through the LL registers; the driver only owns the
 * channels.
 *
 * While unmuted the DACs stay powered. Silence is a steady mid-scale level,
 * the cosine's own centre, and a tone only connects the generator, so the
 * amplifier input never jumps: powering the DACs up and down for every short
 * tick left the amplifier settling longer than the tick lasted. */
#include "odroidgo.h"
#include "driver/dac_cosine.h"
#include "esp_clk_tree.h"
#include "esp_rom_sys.h"
#include "hal/dac_ll.h"
#include <math.h>

#define MID_SCALE 128

static dac_cosine_handle_t chan[2];
static uint32_t rc_fast_hz;
static unsigned volume = 2, playing_hz;
static bool powered;

/* Level 0 is mute; 1..4 = -18, -12, -6, 0 dB. */
static const dac_cosine_atten_t atten[OG_TONE_VOLUMES] = {
    DAC_COSINE_ATTEN_DB_18, DAC_COSINE_ATTEN_DB_18, DAC_COSINE_ATTEN_DB_12,
    DAC_COSINE_ATTEN_DB_6, DAC_COSINE_ATTEN_DB_0,
};

static dac_channel_t ch(int i) { return i ? DAC_CHAN_1 : DAC_CHAN_0; }

static void power(bool on) {
    if (on == powered) return;
    for (int i = 0; i < 2; i++) {
        if (on) {
            dac_cosine_start(chan[i]);
            dac_ll_update_output_value(ch(i), MID_SCALE); /* also detaches the generator */
        } else {
            dac_cosine_stop(chan[i]);
        }
    }
    powered = on;
}

void og_tone_init(void) {
    for (int i = 0; i < 2; i++) {
        const dac_cosine_config_t cfg = {
            .chan_id = ch(i),
            .freq_hz = 1000,
            .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
            .atten = atten[volume],
            .phase = i ? DAC_COSINE_PHASE_180 : DAC_COSINE_PHASE_0,
            .offset = 0,
            .flags.force_set_freq = true,
        };
        ESP_ERROR_CHECK(dac_cosine_new_channel(&cfg, &chan[i]));
    }
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_RC_FAST, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &rc_fast_hz);
    if (!rc_fast_hz) rc_fast_hz = SOC_CLK_RC_FAST_FREQ_APPROX;
}

void og_tone(unsigned hz) {
    if (!volume) hz = 0;
    if (hz && hz < 130) hz = 130; /* generator floor with the default RTC clock */
    if (hz == playing_hz) return;
    if (hz) {
        power(true);
        dac_ll_cw_set_freq(hz, rc_fast_hz);
        if (!playing_hz)
            for (int i = 0; i < 2; i++) dac_ll_cw_enable_channel(ch(i), true);
    } else if (powered) {
        for (int i = 0; i < 2; i++) dac_ll_update_output_value(ch(i), MID_SCALE);
    }
    playing_hz = hz;
}

unsigned og_tone_hz(void) { return playing_hz; }

/* Pulse amplitude in DAC steps: 2 (barely audible) .. 40 (about the cosine
 * at -18 dB, the comfortable level), log-spaced. */
#define CLICK_MIN 2.0f
#define CLICK_MAX 40.0f
#define CLICK_HALF_US 150

void og_click(float loudness) {
    if (!volume || playing_hz) return;
    loudness = loudness < 0 ? 0 : loudness > 1 ? 1 : loudness;
    const int a = (int)(CLICK_MIN * powf(CLICK_MAX / CLICK_MIN, loudness) + 0.5f);
    power(true);
    /* Up, down, back to the middle: no net DC step for the amplifier. */
    dac_ll_update_output_value(ch(0), MID_SCALE + a);
    dac_ll_update_output_value(ch(1), MID_SCALE - a);
    esp_rom_delay_us(CLICK_HALF_US);
    dac_ll_update_output_value(ch(0), MID_SCALE - a);
    dac_ll_update_output_value(ch(1), MID_SCALE + a);
    esp_rom_delay_us(CLICK_HALF_US);
    for (int i = 0; i < 2; i++) dac_ll_update_output_value(ch(i), MID_SCALE);
}

void og_tone_volume(unsigned level) {
    if (level >= OG_TONE_VOLUMES) level = OG_TONE_VOLUMES - 1;
    volume = level;
    for (int i = 0; i < 2; i++) dac_ll_cw_set_atten(ch(i), atten[level]);
    if (!level) {
        og_tone(0);
        power(false); /* muted: DACs off, no idle hiss */
    }
}
