/* ODROID-GO on-device receiver UI. MENU cycles SPEC -> WIFI or ZIGBEE (VOLUME
 * flips between those two) -> HUNT:
 *   SPEC    spectrum + waterfall around a centre frequency (80/40/16 MHz span)
 *   WIFI    Wi-Fi channels 1-13: level now, decaying hold, session max and
 *           airtime (share of time above the floor), best of 1/6/11
 *   ZIGBEE  802.15.4 channels 11-26: the same levels plus burst activity
 *   HUNT    one Zigbee/Wi-Fi channel or frequency: big level above the noise
 *           floor, bar, 32 s history, a tone whose pitch follows the level
 *           (or a tick per burst)
 * Levels come from burst-aware band power (zb.c). Snapshot captures only:
 * the original ESP32 has no continuous IQ path. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "odroidgo.h"
#include "burst_serial.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define W OG_LCD_W
#define H OG_LCD_H
#define SH OG_STRIP_H

#define RGB565(r, g, b) (((r) & 0xf8) << 8 | ((g) & 0xfc) << 3 | (b) >> 3)
#define RGB(r, g, b) ((uint16_t)((RGB565(r, g, b) >> 8) | (RGB565(r, g, b) << 8)))
#define C_BLACK RGB(0, 0, 0)
#define C_WHITE RGB(255, 255, 255)
#define C_TEXT RGB(220, 220, 220)
#define C_DIM RGB(110, 110, 110)
#define C_GRID RGB(40, 40, 40)
#define C_BAR RGB(24, 24, 48)
#define C_WIFI RGB(0, 0, 44)
#define C_ZB RGB(0, 90, 0)
#define C_WIFI_MARK RGB(0, 70, 110)
#define C_YELLOW RGB(255, 220, 0)
#define C_RED RGB(255, 40, 40)
#define C_GREEN RGB(40, 220, 40)
#define C_CYAN RGB(0, 200, 230)
#define C_ORANGE RGB(255, 140, 0)

#define CAP_N 16128u          /* 63 x 256: ~0.2 ms at 80 MS/s, ~1 ms at 16 MS/s */
#define BATCH_US 50000        /* captures between redraws on ZIGBEE and HUNT */
#define DETECT_DB 6.0f        /* burst detection threshold above the floor */
#define SPEC_BATCH_US 30000   /* snapshots per SPEC frame: more air per waterfall row */
#define SPEC_MAX_SNAPS 16
#define FREQ_MIN 1842u        /* tuning range the UI offers, MHz */
#define FREQ_MAX 2700u
#define BW_80 67              /* analog filter per span, MHz */
#define BW_40 36
#define BW_16 14

enum { PAGE_SPEC, PAGE_WIFI, PAGE_ZB, PAGE_HUNT, PAGES };
static const char *const page_name[PAGES] = {"SPEC", "WIFI", "ZIGBEE", "HUNT"};

/* Settings */
static int page;
static int survey_page = PAGE_WIFI; /* MENU's middle stop: WIFI or ZIGBEE, VOLUME flips */
static bool agc = true;            /* SPEC: hardware AGC */
static bool auto_gain = true;      /* ZIGBEE and HUNT: auto-ranged manual gain */
static int manual_gain = 50;
static bool sound = true;          /* speaker on at the lowest level, or muted */
enum { MARK_ZB, MARK_WIFI, MARK_OFF, MARKS };
static int marks = MARK_ZB;        /* SPEC channel overlay */
static unsigned spec_centre = 2440, span; /* span: index into sv_spans */
static bool frozen;
/* SPEC's upper half: LIVE (filled trace), PEAK (plus falling peak hold) or
 * GLOW (digital phosphor: every snapshot adds some intensity under its trace and
 * the picture fades to black, so the usual level glows hot and rare
 * excursions stay faint). */
enum { VIEW_LIVE, VIEW_PEAK, VIEW_GLOW, VIEWS };
static const char *const view_name[VIEWS] = {"LIVE", "PEAK", "GLOW"};
static int view = VIEW_GLOW;      /* GLOW by default: the signature view */
/* SPEC LO hopping: each frame the LO lands a random whole number of MHz off
 * the centre and the snapshots are drawn at their true frequencies. Real
 * signals keep their columns; spurs tied to the LO (DC, IQ images, the 5/6
 * mode's comb) land somewhere else every frame and smear out. */
static const int hop_mhz[] = {0, 1, 5, 10};
#define HOPS ((int)(sizeof(hop_mhz) / sizeof(hop_mhz[0])))
static int hop;
static int glow_fill = 9;          /* added per snapshot, of 255 */
static int glow_ms = 1200;         /* fade time constant */
enum { TGT_ZB, TGT_WIFI, TGT_MHZ, TGTS };
static int hunt_kind = TGT_ZB;     /* what HUNT follows */
static int hunt_ch = 15;           /* Zigbee 11..26 or Wi-Fi 1..13 */
static unsigned hunt_mhz = 2425;   /* TGT_MHZ, 1 MHz steps */
static int tone_mode = 2;          /* 0 off, 1 pitch, 2 tick (Geiger, default) */

/* Shared state */
static unsigned held_prev, released, swallow; /* swallow: keys whose release must not act */
static int64_t repeat_at, held_since, splash_until, battery_at, click_at;
static unsigned battery_mv;
/* No charge status: the charger drives only its LED, and the USB-UART
 * bridge's TX line stays high even when USB is unplugged. */

/* LiPo (LCO) under the board's light load: the curve has a slope down to
 * about 3.6 V, then the knee. Tabulated, interpolated; good to about 10 %. */
static unsigned battery_percent(unsigned mv) {
    static const uint16_t curve[][2] = {
        {4200, 100}, {4100, 90}, {4000, 80}, {3920, 70}, {3850, 60}, {3800, 50},
        {3760, 40}, {3720, 30}, {3680, 20}, {3600, 10}, {3450, 5}, {3300, 0},
    };
    if (mv >= curve[0][0]) return 100;
    for (unsigned i = 1; i < sizeof(curve) / sizeof(curve[0]); i++)
        if (mv >= curve[i][0])
            return curve[i][1] + (mv - curve[i][0]) * (curve[i - 1][1] - curve[i][1]) / (curve[i - 1][0] - curve[i][0]);
    return 0;
}

static void battery_update(void) {
    unsigned mv = og_battery_mv();
    if (mv) battery_mv = battery_mv ? (battery_mv * 3 + mv) / 4 : mv;
}
static bool host_shown, clip, capture_failed;
static float scratch[SV_BINS];

static int zb_channel_mhz(int ch) { return 2405 + 5 * (ch - 11); }

/* ---- gain ranging (ZIGBEE, HUNT) -------------------------------------------- */

/* Levels are referred to gain 50 so they stay comparable while ranging
 * (gain curve in sdr_view.h). */
#define GAIN_REF SV_GAIN_REF
#define GAIN_MIN SV_GAIN_MIN
#define GAIN_DB_STEP SV_GAIN_DB_STEP
static int64_t range_at;
static float clip_rate;

static float gain_offset(int gain) { return GAIN_DB_STEP * (gain - GAIN_REF); }

/* Ranging follows the share of clipped captures, not single ones: occasional
 * strong bursts of other traffic are dropped (the caller skips clipped
 * captures) while the gain stays high enough for weak signals. A target that
 * clips most captures pulls the gain down 3 steps at a time. */
static void autorange(const zb_result_t *r, int64_t now) {
    if (!auto_gain) return;
    clip_rate += 0.1f * ((r->clip ? 1.0f : 0.0f) - clip_rate);
    int g = manual_gain;
    if (clip_rate > 0.25f && now - range_at > 300000) g -= 3;
    else if (clip_rate < 0.03f && now - range_at > 1500000) g += 3;
    if (g < GAIN_MIN) g = GAIN_MIN;
    if (g > GAIN_REF) g = GAIN_REF;
    if (g != manual_gain) {
        manual_gain = g;
        sdr_local_set_gain(g);
        range_at = now;
    }
}

/* Quiet noise floor referred to gain 50: follows drops quickly and rises
 * slowly, so bursts of other traffic do not lift it. Below gain 46 the ADC's
 * own noise dominates and reads high, so there it may only fall. */
static void track_floor(float *ref, float floor_ref, int gain) {
    if (*ref < -150) *ref = floor_ref;
    else if (floor_ref < *ref) *ref += 0.2f * (floor_ref - *ref);
    else if (gain >= 46) *ref += 0.01f * (floor_ref - *ref);
}
static int wifi_channel_mhz(int ch) { return ch == 14 ? 2484 : 2407 + 5 * ch; }

/* ---- drawing helpers ------------------------------------------------------ */

typedef void (*strip_fn)(uint16_t *s, int y0, int h);

static void render(strip_fn fn, int top, int bottom) {
    for (int y = top; y < bottom; y += SH) {
        int h = bottom - y < SH ? bottom - y : SH;
        uint16_t *s = og_lcd_strip();
        fn(s, y, h);
        og_lcd_blit(0, y, W, h, s);
    }
}

static void text(uint16_t *s, int y0, int h, int x, int y, const char *str, uint16_t c, int scale) {
    sv_text(s, W, h, x, y - y0, str, c, scale);
}

static void rect(uint16_t *s, int y0, int h, int x, int y, int w, int rh, uint16_t c) {
    int ya = y > y0 ? y : y0, yb = y + rh < y0 + h ? y + rh : y0 + h;
    if (x < 0) {w += x; x = 0;}
    if (x + w > W) w = W - x;
    for (int yy = ya; yy < yb; yy++)
        for (int xx = x; xx < x + w; xx++) s[(yy - y0) * W + xx] = c;
}

static uint16_t level_colour(float snr) {
    return snr < DETECT_DB ? C_DIM : snr < 15 ? C_GREEN : snr < 25 ? C_YELLOW : C_RED;
}

static void draw_status(uint16_t *s, int y0, int h) {
    if (y0 > 15) return;
    rect(s, y0, h, 0, 0, W, 16, C_BAR);
    char t[48];
    text(s, y0, h, 4, 4, page_name[page], C_YELLOW, 1);
    if (page == PAGE_SPEC && agc) snprintf(t, sizeof(t), "AGC");
    else snprintf(t, sizeof(t), page != PAGE_SPEC && auto_gain ? "A%d" : "G%d", manual_gain);
    text(s, y0, h, 64, 4, t, C_TEXT, 1);
    if (clip) text(s, y0, h, 242, 4, "CLIP", C_RED, 1);
    if (capture_failed) text(s, y0, h, 242, 4, "CERR", C_RED, 1);
    if (page == PAGE_SPEC) {
        static const char *const mark_name[MARKS] = {"ZB", "WIFI", "--"};
        text(s, y0, h, 210, 4, mark_name[marks], marks == MARK_ZB ? C_GREEN : marks == MARK_WIFI ? C_CYAN : C_DIM, 1);
    } else if (page == PAGE_HUNT) {
        text(s, y0, h, 210, 4, sound ? "SND" : "MUTE", sound ? C_TEXT : C_DIM, 1);
    }
    if (battery_mv) {
        unsigned pct = battery_percent(battery_mv);
        snprintf(t, sizeof(t), "%3u%%", pct);
        text(s, y0, h, W - 6 * 4 - 4, 4, t, pct <= 10 ? C_RED : C_TEXT, 1);
    }
}

/* ---- SPEC ------------------------------------------------------------------ */

#define SPEC_Y 16
#define SPEC_H 112                          /* plot rows above the waterfall */
#define WF_ROWS (H - SPEC_Y - SPEC_H - 16)  /* below the 16-row axis */
#define GLOW_H (SPEC_H + WF_ROWS)           /* GLOW takes the waterfall's rows too */

static float cur[W], hold[W];
static int16_t cur_row[W], hold_row[W];
static uint16_t col_bg[W];
static uint8_t col_mark[W]; /* channel number whose centre falls in the column */
static uint8_t *wf;
static int wf_head, spec_frames;
/* Waterfall smoothing: each shown row is the mean (in dB, as LUT indices are
 * linear in dB) of the last wf_avg frames, still one row per frame. */
#define WF_AVG_MAX 4
static uint8_t wf_raw[WF_AVG_MAX][W];
static int wf_raw_head, wf_raw_n, wf_avg = 1;
static uint8_t wf_got[WF_ROWS];   /* diagnostics: snapshots behind each waterfall row */
static int16_t wf_db10[WF_ROWS];  /* diagnostics: row mean level, 0.1 dB */
static unsigned spec_snap_us, spec_frame_us; /* diagnostics: one snapshot incl. drawing prep, one frame */
static int64_t spec_frame_at;
/* Fixed scale, dBFS at gain 50 (sdr_view.c compensates every sample's gain):
 * fitted once at start and on span changes, otherwise moved only by hand. */
static int8_t ref = -20, range = 70;
static bool fit_pending = true;
static int64_t scale_note_until, hop_note_until;
static uint16_t lut[256];
/* Persistence intensities. The upper SPEC_H rows live in chunks (the heap is
 * split over several SRAM regions and rarely holds 36 KB in one piece);
 * full-screen GLOW borrows the waterfall's buffer for the rows below. */
#define GLOW_CHUNKS 8
#define GLOW_ROWS (SPEC_H / GLOW_CHUNKS)
_Static_assert(SPEC_H % GLOW_CHUNKS == 0, "whole rows per chunk");
static uint8_t *glow_chunk[GLOW_CHUNKS];
static bool glow;                  /* all chunks allocated */
static int8_t glow_ref, glow_range;
static int64_t glow_at, tweak_until;

static bool glow_shown(void) { return view == VIEW_GLOW && glow; }
static int plot_h(void) { return glow_shown() ? GLOW_H : SPEC_H; }
static inline uint8_t *glow_row(int r) {
    return r < SPEC_H ? glow_chunk[r / GLOW_ROWS] + (r % GLOW_ROWS) * W : wf + (r - SPEC_H) * W;
}

static void glow_clear(void) {
    if (glow) for (int r = 0; r < plot_h(); r++) memset(glow_row(r), 0, W);
    glow_ref = ref;
    glow_range = range;
}

/* The scale moved: move every column's intensities to the rows that now show
 * the same level. Levels below the old bottom were filled at least as often
 * as the old bottom row (fills run from the trace down); levels above the old
 * top were never reached. */
static void glow_rescale(void) {
    const int gh = plot_h();
    uint8_t col[GLOW_H];
    for (int x = 0; x < W; x++) {
        for (int r = 0; r < gh; r++) col[r] = glow_row(r)[x];
        for (int r = 0; r < gh; r++) {
            float lvl = ref - (float)r * range / (gh - 1);
            int o = (int)lroundf((glow_ref - lvl) * (gh - 1) / glow_range);
            glow_row(r)[x] = o < 0 ? 0 : col[o >= gh ? gh - 1 : o];
        }
    }
    glow_ref = ref;
    glow_range = range;
}

static void glow_fade(int64_t now) {
    if (glow_at) {
        unsigned m = (unsigned)(256 * expf(-(now - glow_at) / (glow_ms * 1000.0f)));
        if (m > 255) m = 255;
        for (int r = 0; r < plot_h(); r++)
            for (uint8_t *p = glow_row(r), *e = p + W; p < e; p++) *p = (uint8_t)((*p * m) >> 8);
    }
    glow_at = now;
}

static void glow_add(const float *col) {
    const int gh = plot_h();
    for (int x = 0; x < W; x++) {
        float r = (ref - col[x]) * (gh - 1) / range;
        for (int row = r < 0 ? 0 : r > gh ? gh : (int)r; row < gh; row++) {
            uint8_t *p = glow_row(row) + x;
            int v = *p + glow_fill;
            *p = v > 255 ? 255 : (uint8_t)v;
        }
    }
}

/* The ESP32's own 40 MHz crystal harmonics (2400, 2440, 2480 MHz) and the
 * residue of the LO's DC leak are not on the air: bridge them with the
 * neighbouring bins. A real carrier within 0.25 MHz of them is hidden too. */
static void blank_spurs(float *db, unsigned lo) {
    const float bin = (float)sv_spans[span].msps / SV_FFT_N;
    for (int k = 0; k < SV_BINS; k++) {
        float f = sv_bin_mhz(lo, span, k), h = fabsf(f - 40 * roundf(f / 40));
        if (h > 0.25f && fabsf(f - lo) > 1.5f * bin) continue;
        int a = k, b = k;
        while (a > 0) {
            float fa = sv_bin_mhz(lo, span, --a);
            if (fabsf(fa - 40 * roundf(fa / 40)) > 0.25f && fabsf(fa - lo) > 1.5f * bin) break;
        }
        while (b < SV_BINS - 1) {
            float fb = sv_bin_mhz(lo, span, ++b);
            if (fabsf(fb - 40 * roundf(fb / 40)) > 0.25f && fabsf(fb - lo) > 1.5f * bin) break;
        }
        db[k] = 0.5f * (db[a] + db[b]);
    }
}

static int span_bw(unsigned sp) { return sv_spans[sp].msps == 80 ? BW_80 : sv_spans[sp].msps == 40 ? BW_40 : BW_16; }
static float col_mhz(int x) { return sv_bin_mhz(spec_centre, span, (x + 0.5f) * SV_BINS / W - 0.5f); }

static void spec_layout(void) {
    for (int x = 0; x < W; x++) {
        float f = col_mhz(x);
        col_bg[x] = C_BLACK;
        if (marks == MARK_WIFI)
            for (int ch = 1; ch <= 11; ch += 5)
                if (fabsf(f - wifi_channel_mhz(ch)) <= 10) col_bg[x] = C_WIFI;
        col_mark[x] = 0;
    }
    if (marks == MARK_OFF) return;
    const bool zb = marks == MARK_ZB;
    for (int ch = zb ? 11 : 1; ch <= (zb ? 26 : 13); ch++) {
        int f = zb ? zb_channel_mhz(ch) : wifi_channel_mhz(ch);
        int x = (int)(sv_mhz_bin(spec_centre, span, f) * W / SV_BINS);
        if (x >= 0 && x < W) col_mark[x] = (uint8_t)ch;
    }
}

/* Fit the scale to what is on the air now: 5 dB over the highest peak, 10 dB
 * under the median level. */
static void spec_fit(void) {
    float top = -200;
    for (int x = 0; x < W; x++) if (hold[x] > top) top = hold[x];
    int r = (int)ceilf((top + 5) / 5) * 5, bottom = (int)floorf((sv_floor(cur, W, scratch) - 10) / 5) * 5;
    r = r < -120 ? -120 : r > 20 ? 20 : r;
    int g = r - bottom < 20 ? 20 : r - bottom > 120 ? 120 : r - bottom;
    ref = (int8_t)r;
    range = (int8_t)g;
}

static void spec_strip(uint16_t *s, int y0, int h) {
    draw_status(s, y0, h);
    char t[32];
    const int ph = plot_h(), axis = SPEC_Y + ph;
    const bool glowing = glow_shown();
    for (int y = y0; y < y0 + h; y++) {
        uint16_t *row = s + (y - y0) * W;
        if (y >= SPEC_Y && y < axis) {
            int r = y - SPEC_Y;
            float lvl = ref - (float)r * range / (ph - 1);
            bool grid = (int)floorf(lvl / 10) != (int)floorf((ref - (float)(r + 1) * range / (ph - 1)) / 10);
            for (int x = 0; x < W; x++) {
                uint16_t c = col_bg[x];
                if (grid || (col_mark[x] && (r & 3) == 0)) c = grid ? C_GRID : marks == MARK_ZB ? C_ZB : C_WIFI_MARK;
                if (glowing) {
                    uint8_t v = glow_row(r)[x];
                    if (v) c = lut[v];
                } else if (r >= cur_row[x]) {
                    unsigned g = 90 + 160 * (SPEC_H - r) / SPEC_H;
                    c = RGB(0, g, g > 200 ? 255 : g + 40);
                }
                if (view == VIEW_PEAK && r == hold_row[x]) c = C_YELLOW;
                row[x] = c;
            }
        } else if (!glowing && y >= axis + 16) {
            const uint8_t *line = wf + ((wf_head - (y - axis - 16) + WF_ROWS) % WF_ROWS) * W;
            for (int x = 0; x < W; x++) row[x] = lut[line[x]];
        } else if (y >= 16) {
            for (int x = 0; x < W; x++) row[x] = C_BLACK;
        }
    }
    snprintf(t, sizeof(t), "%d", ref);
    text(s, y0, h, 2, SPEC_Y + 2, t, C_DIM, 1);
    snprintf(t, sizeof(t), "%d", ref - range);
    text(s, y0, h, 2, axis - 10, t, C_DIM, 1);
    snprintf(t, sizeof(t), "%u MHz  %u MS/s", spec_centre, sv_spans[span].msps);
    text(s, y0, h, 96, 4, t, C_CYAN, 1);
    text(s, y0, h, W - 26, SPEC_Y + 2, view_name[view], C_DIM, 1);
    if (hop) {
        snprintf(t, sizeof(t), "LO~%d", hop_mhz[hop]);
        text(s, y0, h, W - 32 - 6 * (int)strlen(t), SPEC_Y + 2, t, C_CYAN, 1);
    }
    if (frozen) text(s, y0, h, W - 100, SPEC_Y + 2, "HOLD", C_ORANGE, 1);
    const int64_t now = esp_timer_get_time();
    if (glowing && now < tweak_until) {
        snprintf(t, sizeof(t), "fill %d  fade %d.%ds", glow_fill, glow_ms / 1000, glow_ms % 1000 / 100);
        text(s, y0, h, 40, SPEC_Y + 2, t, C_YELLOW, 1);
    }
    if (now < scale_note_until) {
        snprintf(t, sizeof(t), "scale %d .. %d dB", ref - range, ref);
        text(s, y0, h, 40, SPEC_Y + 12, t, C_YELLOW, 1);
    }
    if (now < hop_note_until) {
        if (hop) snprintf(t, sizeof(t), "LO hops +-%d MHz", hop_mhz[hop]);
        else snprintf(t, sizeof(t), "LO hops off");
        text(s, y0, h, 40, SPEC_Y + 22, t, C_YELLOW, 1);
    }
    if (!glowing && wf_avg > 1) {
        snprintf(t, sizeof(t), "WF/%d", wf_avg);
        text(s, y0, h, 2, axis + 18, t, C_WHITE, 1);
    }
    /* Axis: channel numbers above, frequencies below. */
    if (y0 <= axis + 15 && y0 + h > axis) {
        int last = -100;
        for (int x = 0; x < W; x++) if (col_mark[x] && x - last >= 14) {
            snprintf(t, sizeof(t), "%u", col_mark[x]);
            text(s, y0, h, x - (col_mark[x] < 10 ? 3 : 6), axis, t, marks == MARK_ZB ? C_GREEN : C_CYAN, 1);
            last = x;
        }
        snprintf(t, sizeof(t), "%.0f", col_mhz(0));
        text(s, y0, h, 0, axis + 8, t, C_TEXT, 1);
        snprintf(t, sizeof(t), "%u", spec_centre);
        text(s, y0, h, W / 2 - 12, axis + 8, t, C_TEXT, 1);
        snprintf(t, sizeof(t), "%.0f", col_mhz(W - 1));
        text(s, y0, h, W - 24, axis + 8, t, C_TEXT, 1);
    }
}

static void spec_frame(void) {
    const bool glowing = glow_shown();
    if (glowing && (glow_ref != ref || glow_range != range)) glow_rescale();
    if (!frozen) {
        sdr_local_set_bandwidth(span_bw(span));
        sdr_local_set_gain(agc ? -1 : manual_gain);
        int off = 0;
        if (hop) {
            const int d = hop_mhz[hop];
            off = (int)(esp_random() % (unsigned)(2 * d + 1)) - d;
            if ((int)spec_centre + off < (int)FREQ_MIN) off = (int)FREQ_MIN - (int)spec_centre;
            if ((int)spec_centre + off > (int)FREQ_MAX) off = (int)FREQ_MAX - (int)spec_centre;
        }
        const unsigned lo = spec_centre + off;
        /* Bins measured around lo, shown around spec_centre. */
        const int shift = (int)lroundf(off * (float)SV_FFT_N / sv_spans[span].msps);
        sdr_local_tune(lo);
        og_lcd_idle();
        /* Several snapshots per frame: bursty emitters (a microwave oven pulses
         * at twice the mains frequency) land in at least one. The trace shows
         * their mean, the hold and the waterfall their maximum. Columns a
         * hopped LO did not reach this frame stay as they were. */
        static float bins[SV_BINS], moved[SV_BINS], col[W], mean[W], peak[W];
        static uint8_t seen[W];
        for (int x = 0; x < W; x++) {mean[x] = 0; peak[x] = -INFINITY; seen[x] = 0;}
        int got = 0;
        int64_t t0 = esp_timer_get_time();
        if (glowing) glow_fade(t0);
        do {
            const float *db = sv_measure(span, true);
            if (!db) break;
            memcpy(bins, db, sizeof(bins));
            blank_spurs(bins, lo);
            const float *at = bins;
            if (shift) {
                for (int j = 0; j < SV_BINS; j++) {
                    int k = j - shift;
                    moved[j] = k >= 0 && k < SV_BINS ? bins[k] : -INFINITY;
                }
                at = moved;
            }
            sv_resample_max(at, SV_BINS, col, W);
            if (glowing && !fit_pending) glow_add(col); /* -inf columns add nothing */
            for (int x = 0; x < W; x++) {
                if (!(col[x] > -1e30f)) continue;
                mean[x] += col[x];
                seen[x]++;
                if (col[x] > peak[x]) peak[x] = col[x];
            }
            got++;
            spec_snap_us = (unsigned)((esp_timer_get_time() - t0) / got);
        } while (esp_timer_get_time() - t0 < SPEC_BATCH_US && got < SPEC_MAX_SNAPS);
        capture_failed = !got;
        if (got) {
            for (int x = 0; x < W; x++) {
                if (seen[x]) {
                    float m = mean[x] / seen[x];
                    cur[x] = spec_frames ? 0.5f * cur[x] + 0.5f * m : m;
                }
                hold[x] = spec_frames && hold[x] - 0.4f > peak[x] ? hold[x] - 0.4f : peak[x];
            }
            if (fit_pending) {
                spec_fit();
                fit_pending = false;
                scale_note_until = esp_timer_get_time() + 2500000;
            }
            if (!glowing) {
                wf_head = (wf_head + 1) % WF_ROWS;
                wf_raw_head = (wf_raw_head + 1) % WF_AVG_MAX;
                if (wf_raw_n < WF_AVG_MAX) wf_raw_n++;
                uint8_t *raw = wf_raw[wf_raw_head], *line = wf + wf_head * W;
                const int k = wf_avg < wf_raw_n ? wf_avg : wf_raw_n;
                float sum = 0;
                int n = 0;
                for (int x = 0; x < W; x++) {
                    float v = (peak[x] - (ref - range)) * 255 / range;
                    raw[x] = v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v;
                    unsigned a = 0;
                    for (int j = 0; j < k; j++) a += wf_raw[(wf_raw_head - j + WF_AVG_MAX) % WF_AVG_MAX][x];
                    line[x] = (uint8_t)((a + k / 2) / k);
                    if (peak[x] > -1e30f) {sum += peak[x]; n++;}
                }
                wf_got[wf_head] = (uint8_t)got;
                wf_db10[wf_head] = n ? (int16_t)(sum / n * 10) : -9999;
            }
            spec_frames++;
        }
    }
    for (int x = 0; x < W; x++) {
        float r = (ref - cur[x]) * (SPEC_H - 1) / range, p = (ref - hold[x]) * (SPEC_H - 1) / range;
        cur_row[x] = (int16_t)(r < 0 ? 0 : r > SPEC_H ? SPEC_H : r);
        hold_row[x] = (int16_t)(p < 0 ? 0 : p > SPEC_H - 1 ? SPEC_H - 1 : p);
    }
    render(spec_strip, 0, H);
    const int64_t now = esp_timer_get_time();
    spec_frame_us = spec_frame_at ? (unsigned)(now - spec_frame_at) : 0;
    spec_frame_at = now;
}

static void spec_retuned(void) {
    spec_layout();
    spec_frames = 0;
    wf_raw_n = 0;
    for (int x = 0; x < W; x++) cur[x] = hold[x] = -200;
    memset(wf, 0, WF_ROWS * W);
    glow_clear();
}

/* ---- channel surveys: WIFI (1-13) and ZIGBEE (11-26) ------------------------ */

/* Two 80 MS/s windows cover 2.4 GHz; no Wi-Fi or Zigbee channel centre sits
 * on either LO's DC. */
#define LO_A 2422
#define LO_B 2462
#define BAR_Y 40
#define BAR_H 164
#define BAR_DB 42.0f
#define MAX_CH 16

typedef struct {
    float now, hold, max, air, air_sum;
    int64_t hold_at;
    unsigned busy, total;
} chan_t;

typedef struct {
    int first, count;   /* channel numbers first .. first + count - 1 */
    int (*mhz)(int ch);
    float half_mhz;     /* band measured around each centre */
    int split;          /* channels [0, split) from LO_A, the rest from LO_B */
    bool airtime;       /* Wi-Fi: share of time busy; Zigbee: captures with a burst */
    int cursor;
    chan_t ch[MAX_CH];
    float band[MAX_CH];
    float ref;          /* quiet floor at gain 50, dB per bin */
    bool lo_b;
    int64_t since, last;
    /* GLOW view: per channel, how often its level reached each bar row
     * (same persistence as SPEC's, shared fill and fade). */
    bool glow_on;
    int64_t glow_at;
    uint8_t glow[MAX_CH][BAR_H];
} survey_t;

/* Wi-Fi: the central 16 MHz of each 20 MHz channel. */
static survey_t wifi_sv = {.first = 1, .count = 13, .mhz = wifi_channel_mhz, .half_mhz = 8,
                           .split = 7, .airtime = true, .cursor = 6, .glow_on = true};
static survey_t zb_sv = {.first = 11, .count = 16, .mhz = zb_channel_mhz, .half_mhz = 1,
                         .split = 9, .airtime = false, .cursor = 15, .glow_on = true};

static survey_t *survey(void) { return page == PAGE_WIFI ? &wifi_sv : &zb_sv; }

static void survey_reset(survey_t *v) {
    memset(v->ch, 0, sizeof(v->ch));
    memset(v->glow, 0, sizeof(v->glow));
    for (int i = 0; i < v->count; i++) v->band[i] = v->mhz(v->first + i);
    v->since = esp_timer_get_time();
    v->ref = -200;
}

static void chan_update(chan_t *c, float snr, float busy, bool airtime, int64_t now) {
    c->now = snr;
    if (snr >= c->hold) {c->hold = snr; c->hold_at = now + 1500000;}
    if (snr > c->max) c->max = snr;
    c->total++;
    if (airtime) {
        c->air_sum += busy;
        c->air += 0.01f * (busy - c->air); /* ~3 s at the survey's capture rate */
    } else if (snr >= DETECT_DB) {
        c->busy++;
    }
}

/* Session share in tenths of a percent. */
static unsigned chan_share(const survey_t *v, const chan_t *c) {
    if (!c->total) return 0;
    return v->airtime ? (unsigned)(c->air_sum * 1000 / c->total + 0.5f) : (c->busy * 1000 + c->total / 2) / c->total;
}

/* Quietest of the non-overlapping Wi-Fi channels 1, 6 and 11 (recent airtime). */
static int wifi_best(void) {
    int best = 0;
    for (int ch = 1; ch <= 11; ch += 5)
        if (wifi_sv.ch[ch - 1].total && (!best || wifi_sv.ch[ch - 1].air < wifi_sv.ch[best - 1].air)) best = ch;
    return best;
}

static int bar_y(float db) {
    return (int)(BAR_Y + BAR_H - (db < 0 ? 0 : db > BAR_DB ? BAR_DB : db) * BAR_H / BAR_DB);
}

static void survey_glow_fade(survey_t *v, int64_t now) {
    if (v->glow_at) {
        unsigned m = (unsigned)(256 * expf(-(now - v->glow_at) / (glow_ms * 1000.0f)));
        if (m > 255) m = 255;
        for (uint8_t *p = &v->glow[0][0], *e = p + sizeof(v->glow); p < e; p++) *p = (uint8_t)((*p * m) >> 8);
    }
    v->glow_at = now;
}

static void survey_glow_add(survey_t *v, int i, float snr) {
    for (int r = bar_y(snr) - BAR_Y; r < BAR_H; r++) {
        int g = v->glow[i][r] + glow_fill;
        v->glow[i][r] = g > 255 ? 255 : (uint8_t)g;
    }
}

static void survey_strip(uint16_t *s, int y0, int h) {
    const survey_t *v = survey();
    sv_fill(s, W, h, C_BLACK);
    draw_status(s, y0, h);
    char t[96];
    const chan_t *c = &v->ch[v->cursor - v->first];
    unsigned share = chan_share(v, c);
    snprintf(t, sizeof(t), "ch%d %d MHz  now %+.0f hold %+.0f max %+.0f dB  %s %u.%u%%",
             v->cursor, v->mhz(v->cursor), c->now, c->hold, c->max, v->airtime ? "air" : "act",
             share / 10, share % 10);
    if (v->glow_on && esp_timer_get_time() < tweak_until)
        snprintf(t, sizeof(t), "GLOW  fill %d  fade %d.%ds", glow_fill, glow_ms / 1000, glow_ms % 1000 / 100);
    text(s, y0, h, 2, 22, t, v->glow_on && esp_timer_get_time() < tweak_until ? C_YELLOW : C_TEXT, 1);
    text(s, y0, h, 210, 4, v->glow_on ? "GLOW" : "BARS", C_DIM, 1);
    const int cw = W / v->count, x0 = (W - cw * v->count) / 2, best = v->airtime ? wifi_best() : 0;
    for (int i = 0; i < v->count; i++) {
        const int x = x0 + i * cw, chn = v->first + i, f = v->mhz(chn);
        const chan_t *q = &v->ch[i];
        bool shade = false;
        if (v->airtime) shade = chn == 1 || chn == 6 || chn == 11;
        else for (int w = 1; w <= 11; w += 5) if (abs(f - wifi_channel_mhz(w)) <= 10) shade = true;
        rect(s, y0, h, x + 1, BAR_Y, cw - 2, BAR_H, chn == v->cursor ? RGB(50, 50, 70) : shade ? C_WIFI : RGB(14, 14, 14));
        for (int d = 10; d < BAR_DB; d += 10) rect(s, y0, h, x + 1, bar_y(d), cw - 2, 1, C_GRID);
        int top = bar_y(q->now), bw = v->airtime ? cw - 9 : cw - 6;
        if (!v->glow_on) {
            rect(s, y0, h, x + 3, top, bw, BAR_Y + BAR_H - top, level_colour(q->now));
        } else {
            for (int y = y0 > BAR_Y ? y0 : BAR_Y; y < y0 + h && y < BAR_Y + BAR_H; y++) {
                uint8_t g = v->glow[i][y - BAR_Y];
                if (!g) continue;
                uint16_t *row = s + (y - y0) * W, c = lut[g];
                for (int xx = x + 3; xx < x + 3 + bw; xx++) row[xx] = c;
            }
        }
        rect(s, y0, h, x + 2, bar_y(q->hold), cw - 4, 2, C_WHITE);
        rect(s, y0, h, x + 2, bar_y(q->max), cw - 4, 1, C_RED);
        if (v->airtime) {
            /* Recent airtime as a thin orange bar on the right, 0..100 %. */
            int ah = (int)(q->air * BAR_H + 0.5f);
            rect(s, y0, h, x + cw - 5, BAR_Y + BAR_H - ah, 3, ah, C_ORANGE);
            if (chn == best) text(s, y0, h, x + cw / 2 - 12, BAR_Y - 9, "BEST", C_GREEN, 1);
        }
        snprintf(t, sizeof(t), "%d", chn);
        text(s, y0, h, x + cw / 2 - (chn < 10 ? 3 : 6), BAR_Y + BAR_H + 4, t, chn == v->cursor ? C_YELLOW : C_GREEN, 1);
        unsigned p = v->airtime ? (unsigned)(q->air * 100 + 0.5f) : (chan_share(v, q) + 5) / 10;
        snprintf(t, sizeof(t), "%u", p);
        text(s, y0, h, x + cw / 2 - (p < 10 ? 3 : 6), BAR_Y + BAR_H + 16, t, p ? C_ORANGE : C_DIM, 1);
    }
    snprintf(t, sizeof(t), "%ds", (int)((esp_timer_get_time() - v->since) / 1000000));
    text(s, y0, h, 160, 4, t, C_DIM, 1);
}

static void survey_frame(survey_t *v) {
    unsigned lo = v->lo_b ? LO_B : LO_A;
    sdr_local_set_bandwidth(BW_80);
    if (auto_gain && manual_gain > GAIN_REF) manual_gain = GAIN_REF;
    sdr_local_set_gain(manual_gain);
    sdr_local_tune(lo);
    og_lcd_idle();
    int64_t start = esp_timer_get_time();
    clip = false;
    zb_result_t r;
    if (v->glow_on) survey_glow_fade(v, start);
    while (esp_timer_get_time() - start < BATCH_US) {
        const uint32_t *w = sdr_local_capture(CAP_N, 0);
        capture_failed = !w;
        int gain = manual_gain;
        float off = gain_offset(gain);
        float busy_db = v->ref < -150 ? -200 : v->ref + off + DETECT_DB;
        if (!w || !zb_analyze(w, CAP_N, 80, lo, v->band, v->count, v->half_mhz, busy_db, &r)) break;
        int64_t now = esp_timer_get_time();
        float dt = v->last ? (now - v->last) * 1e-6f : 0;
        v->last = now;
        for (int i = 0; i < v->count; i++)
            if (now > v->ch[i].hold_at && v->ch[i].hold > 0) v->ch[i].hold -= 6 * dt;
        clip |= r.clip;
        autorange(&r, now);
        if (r.clip && auto_gain) continue; /* distorted; the next one is ranged */
        track_floor(&v->ref, r.floor - off, gain);
        for (int i = v->lo_b ? v->split : 0; i < (v->lo_b ? v->count : v->split); i++) {
            if (r.peak[i] < -150) continue;
            float snr = r.peak[i] - off - v->ref;
            chan_update(&v->ch[i], snr, r.busy[i], v->airtime, now);
            if (v->glow_on) survey_glow_add(v, i, snr);
        }
    }
    v->lo_b = !v->lo_b;
    render(survey_strip, 0, H);
}

/* ---- HUNT ------------------------------------------------------------------ */

#define HUNT_BIG_Y 40
#define HUNT_BAR_Y 104
#define HIST_Y 128
#define HIST_H (H - HIST_Y - 10)
#define HIST_DB 48.0f
#define HIST_US 100000

static float hunt_now, hunt_hold, hunt_max, hunt_env, hunt_level, hunt_floor, hist_acc;
static int64_t hunt_hold_at, hist_at, hunt_last;
static uint8_t hist[W];
static int hist_head;
static unsigned hunt_bursts, hunt_ticks;
static float hunt_ref = -200; /* quiet floor at gain 50, dB per bin */

static unsigned hunt_target(void) {
    return hunt_kind == TGT_ZB ? (unsigned)zb_channel_mhz(hunt_ch)
         : hunt_kind == TGT_WIFI ? (unsigned)wifi_channel_mhz(hunt_ch) : hunt_mhz;
}

static void hunt_reset(void) {
    hunt_hold = hunt_max = hunt_env = 0;
    memset(hist, 0, sizeof(hist));
    hunt_bursts = 0;
    hunt_ref = -200;
}

static void hunt_strip(uint16_t *s, int y0, int h) {
    sv_fill(s, W, h, C_BLACK);
    draw_status(s, y0, h);
    char t[64];
    if (hunt_kind == TGT_ZB) snprintf(t, sizeof(t), "Zigbee ch%d  %u MHz", hunt_ch, hunt_target());
    else if (hunt_kind == TGT_WIFI) snprintf(t, sizeof(t), "Wi-Fi ch%d  %u MHz", hunt_ch, hunt_target());
    else snprintf(t, sizeof(t), "%u MHz  (2 MHz band)", hunt_mhz);
    text(s, y0, h, 4, 22, t, C_CYAN, 1);
    static const char *const modes[] = {"tone off", "tone: pitch", "tone: tick"};
    text(s, y0, h, 220, 22, modes[tone_mode], C_DIM, 1);
    snprintf(t, sizeof(t), "%2.0f", hunt_hold < 0 ? 0.0f : hunt_hold);
    text(s, y0, h, 8, HUNT_BIG_Y, t, level_colour(hunt_hold), 7);
    text(s, y0, h, 100, HUNT_BIG_Y + 40, "dB", C_DIM, 2);
    snprintf(t, sizeof(t), "now   %+5.1f dB", hunt_now);
    text(s, y0, h, 160, HUNT_BIG_Y, t, C_TEXT, 1);
    snprintf(t, sizeof(t), "max   %+5.1f dB", hunt_max);
    text(s, y0, h, 160, HUNT_BIG_Y + 12, t, C_RED, 1);
    snprintf(t, sizeof(t), "level %6.1f dBFS@G50", hunt_level);
    text(s, y0, h, 160, HUNT_BIG_Y + 24, t, C_TEXT, 1);
    snprintf(t, sizeof(t), "floor %6.1f", hunt_floor);
    text(s, y0, h, 160, HUNT_BIG_Y + 36, t, C_DIM, 1);
    snprintf(t, sizeof(t), "bursts %u", hunt_bursts);
    text(s, y0, h, 160, HUNT_BIG_Y + 48, t, C_ORANGE, 1);
    /* Level bar, 0..50 dB above the floor. */
    rect(s, y0, h, 0, HUNT_BAR_Y, W, 14, RGB(20, 20, 20));
    int bw = (int)(hunt_env < 0 ? 0 : hunt_env > 50 ? W : hunt_env * W / 50);
    rect(s, y0, h, 0, HUNT_BAR_Y, bw, 14, level_colour(hunt_env));
    int hx = (int)(hunt_hold < 0 ? 0 : hunt_hold > 50 ? W - 2 : hunt_hold * W / 50);
    rect(s, y0, h, hx, HUNT_BAR_Y - 2, 2, 18, C_WHITE);
    int mx = (int)(hunt_max < 0 ? 0 : hunt_max > 50 ? W - 1 : hunt_max * W / 50);
    rect(s, y0, h, mx, HUNT_BAR_Y - 2, 1, 18, C_RED);
    /* History: one column per 100 ms, newest on the right. */
    rect(s, y0, h, 0, HIST_Y, W, HIST_H, RGB(10, 10, 16));
    for (int d = 10; d < HIST_DB; d += 10)
        rect(s, y0, h, 0, HIST_Y + HIST_H - (int)(d * HIST_H / HIST_DB), W, 1, C_GRID);
    rect(s, y0, h, 0, HIST_Y + HIST_H - (int)(DETECT_DB * HIST_H / HIST_DB), W, 1, RGB(60, 30, 0));
    for (int x = 0; x < W; x++) {
        uint8_t v = hist[(hist_head + 1 + x) % W];
        if (!v) continue;
        int bh = v * HIST_H / 255;
        rect(s, y0, h, x, HIST_Y + HIST_H - bh, 1, bh, level_colour(v * HIST_DB / 255));
    }
    text(s, y0, h, 2, H - 9, "-32 s", C_DIM, 1);
    text(s, y0, h, W - 24, H - 9, "now", C_DIM, 1);
}

static void hunt_tone(bool burst, float snr, int64_t now) {
    if (tone_mode == 1) {
        float level = hunt_env;
        og_tone(level >= 4 ? (unsigned)(220 * powf(2, level / 12)) : 0);
    } else if (tone_mode == 2) {
        /* Geiger counter: one click per caught burst, at most one per 25 ms;
         * the rate tells how busy the channel is, the loudness how strong
         * the burst was (threshold .. 40 dB over the floor). */
        og_tone(0);
        if (burst && now - click_at >= 25000) {
            og_click((snr - DETECT_DB) / (40 - DETECT_DB));
            click_at = now;
            hunt_ticks++;
        }
    } else {
        og_tone(0);
    }
}

static void hunt_frame(void) {
    /* Narrow targets: 16 MS/s, LO 4 MHz above. A Wi-Fi channel's central
     * 16 MHz: 40 MS/s, LO 10 MHz above. Either way clear of the LO's DC. */
    const bool wide = hunt_kind == TGT_WIFI;
    const unsigned target = hunt_target(), lo = target + (wide ? 10 : 4), msps = wide ? 40 : 16;
    sdr_local_set_bandwidth(wide ? BW_40 : BW_16);
    if (auto_gain && manual_gain > GAIN_REF) manual_gain = GAIN_REF;
    sdr_local_set_gain(manual_gain);
    sdr_local_tune(lo);
    og_lcd_idle();
    float band = (float)target;
    int64_t start = esp_timer_get_time();
    clip = false;
    zb_result_t r;
    while (esp_timer_get_time() - start < BATCH_US) {
        const uint32_t *w = sdr_local_capture(CAP_N, wide ? 1 : 6);
        capture_failed = !w;
        if (!w || !zb_analyze(w, CAP_N, msps, lo, &band, 1, wide ? 8.0f : 1.0f, -200, &r)) break;
        int64_t now = esp_timer_get_time();
        float dt = hunt_last ? (now - hunt_last) * 1e-6f : 0;
        hunt_last = now;
        clip |= r.clip;
        int gain = manual_gain;
        float off = gain_offset(gain);
        autorange(&r, now);
        if (r.clip && auto_gain) continue; /* distorted; the next one is ranged */
        track_floor(&hunt_ref, r.floor - off, gain);
        float snr = r.peak[0] - off - hunt_ref;
        bool burst = snr >= DETECT_DB;
        hunt_now = snr;
        hunt_level = r.peak[0] - off;
        hunt_floor = hunt_ref;
        hunt_env = snr > hunt_env - 15 * dt ? snr : hunt_env - 15 * dt;
        if (snr >= hunt_hold) {hunt_hold = snr; hunt_hold_at = now + 1000000;}
        else if (now > hunt_hold_at) hunt_hold -= 10 * dt;
        if (snr > hunt_max) hunt_max = snr;
        if (snr > hist_acc) hist_acc = snr;
        if (burst) hunt_bursts++;
        if (now >= hist_at) {
            hist_head = (hist_head + 1) % W;
            float v = hist_acc * 255 / HIST_DB;
            hist[hist_head] = v < 1 ? 0 : v > 255 ? 255 : (uint8_t)v;
            hist_acc = 0;
            hist_at = now + HIST_US;
        }
        hunt_tone(burst, snr, now);
    }
    render(hunt_strip, 0, H);
    hunt_tone(false, 0, esp_timer_get_time());
}

/* ---- splash, host, input --------------------------------------------------- */

static void splash_strip(uint16_t *s, int y0, int h) {
    sv_fill(s, W, h, C_BLACK);
    text(s, y0, h, 40, 16, "ESP-SDR", C_CYAN, 4);
    text(s, y0, h, 210, 30, "ODROID-GO", C_YELLOW, 1);
    char t[32];
    snprintf(t, sizeof(t), "heap %uK/%uK", (unsigned)(esp_get_free_heap_size() / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024));
    text(s, y0, h, 210, 42, t, C_DIM, 1);
    snprintf(t, sizeof(t), "bat %u.%02uV %u%%", battery_mv / 1000, battery_mv % 1000 / 10,
             battery_percent(battery_mv));
    text(s, y0, h, 210, 4, t, C_DIM, 1);
    static const char *const lines[] = {
        "2.4 GHz receiver on the ESP32's own Wi-Fi radio",
        "",
        "MENU    page: SPEC / WIFI or ZIGBEE / HUNT",
        "<- ->   tune / channel     UP DOWN  gain",
        "A       SPEC: span  WIFI/ZB: hunt it  HUNT: target",
        "B       SPEC, WIFI/ZB: view     HUNT: tone",
        "B+pad   GLOW: <- -> fill   UP DOWN fade",
        "START   SPEC: pause   WIFI/ZB, HUNT: reset",
        "START+  UP/DN top <-/-> range A fit B LO hops",
        "START+VOLUME  waterfall average 1/2/3/4 frames",
        "SELECT  SPEC: AGC   other pages: auto gain",
        "VOLUME  SPEC: marks  WIFI<->ZB  HUNT: sound",
        "",
        "USB: still works with the ESP-SDR web viewer",
        "",
        "            press any button",
    };
    for (unsigned i = 0; i < sizeof(lines) / sizeof(lines[0]); i++)
        text(s, y0, h, 14, 56 + 11 * (int)i, lines[i], i == 0 ? C_DIM : C_TEXT, 1);
}

static void error_strip(uint16_t *s, int y0, int h) {
    sv_fill(s, W, h, C_RED);
    text(s, y0, h, 4, 4, "UI INIT FAILED: OUT OF MEMORY", C_WHITE, 1);
}

static void host_strip(uint16_t *s, int y0, int h) {
    sv_fill(s, W, h, C_BAR);
    text(s, y0, h, 4, 4, "USB HOST IN CONTROL", C_ORANGE, 1);
}

static unsigned pressed(void) {
    unsigned held = og_buttons(), now = held & ~held_prev;
    const unsigned rep = OG_UP | OG_DOWN | OG_LEFT | OG_RIGHT;
    int64_t t = esp_timer_get_time();
    if (now & rep) {repeat_at = t + 350000; held_since = t;}
    else if ((held & rep) && t >= repeat_at) {now |= held & rep; repeat_at = t + 90000;}
    released = held_prev & ~held & ~swallow;
    swallow &= held;
    held_prev = held;
    return now;
}

/* A D-pad key held for over a second repeats with coarse steps. */
static bool held_long(void) {
    return (held_prev & (OG_UP | OG_DOWN | OG_LEFT | OG_RIGHT)) && esp_timer_get_time() - held_since > 1000000;
}

static void set_gain(int g) {
    unsigned max = sdr_local_gain_max();
    manual_gain = g < 0 ? 0 : g > (int)max ? (int)max : g;
}

/* SPEC: B acts on release; held, it turns the D-pad into the GLOW knobs. */
static unsigned glow_keys(unsigned k) {
    static bool chord;
    const unsigned pad = OG_UP | OG_DOWN | OG_LEFT | OG_RIGHT;
    if (k & OG_B) chord = false;
    if ((held_prev & OG_B) && (k & pad)) {
        chord = true;
        if (k & OG_UP) glow_ms += glow_ms / 2;
        if (k & OG_DOWN) glow_ms -= glow_ms / 3;
        if (k & OG_RIGHT) glow_fill += glow_fill / 2 > 1 ? glow_fill / 2 : 1;
        if (k & OG_LEFT) glow_fill -= glow_fill / 3 > 1 ? glow_fill / 3 : 1;
        glow_ms = glow_ms < 100 ? 100 : glow_ms > 20000 ? 20000 : glow_ms;
        glow_fill = glow_fill < 1 ? 1 : glow_fill > 128 ? 128 : glow_fill;
        if (page != PAGE_SPEC) {
            survey_t *v = survey();
            if (!v->glow_on) {v->glow_on = true; memset(v->glow, 0, sizeof(v->glow));}
        } else if (view != VIEW_GLOW && glow) {
            view = VIEW_GLOW;
            glow_clear(); /* includes the waterfall rows */
        }
        tweak_until = esp_timer_get_time() + 2500000;
        k &= ~pad;
    }
    if ((released & OG_B) && !chord && page != PAGE_SPEC) {
        survey_t *v = survey();
        v->glow_on = !v->glow_on;
        memset(v->glow, 0, sizeof(v->glow));
        v->glow_at = 0;
    } else if ((released & OG_B) && !chord) {
        view = (view + 1) % VIEWS;
        if (view == VIEW_GLOW && !glow) view = VIEW_LIVE;
        memset(wf, 0, WF_ROWS * W); /* GLOW borrows the waterfall's buffer */
        glow_clear();
    }
    return k & ~OG_B;
}

/* SPEC: START acts on release (pause); held, the D-pad moves the fixed scale,
 * A fits it to the current signal and B cycles the LO hopping. */
static bool start_chord;
static unsigned scale_keys(unsigned k) {
    const unsigned pad = OG_UP | OG_DOWN | OG_LEFT | OG_RIGHT;
    if (k & OG_START) start_chord = false;
    if ((held_prev & OG_START) && (k & OG_VOLUME)) {
        start_chord = true;
        wf_avg = wf_avg % WF_AVG_MAX + 1;
        k &= ~OG_VOLUME;
    }
    if ((held_prev & OG_START) && (k & OG_B)) {
        start_chord = true;
        hop = (hop + 1) % HOPS;
        hop_note_until = esp_timer_get_time() + 2500000;
        glow_clear();
        swallow |= OG_B; /* its release must not change the view */
        k &= ~OG_B;
    }
    if ((held_prev & OG_START) && (k & (pad | OG_A))) {
        start_chord = true;
        int r = ref, g = range;
        if (k & OG_UP) r += 5;
        if (k & OG_DOWN) r -= 5;
        if (k & OG_RIGHT) g += 10;
        if (k & OG_LEFT) g -= 10;
        ref = (int8_t)(r < -120 ? -120 : r > 20 ? 20 : r);
        range = (int8_t)(g < 20 ? 20 : g > 120 ? 120 : g);
        if (k & OG_A) fit_pending = true;
        scale_note_until = esp_timer_get_time() + 2500000;
        k &= ~(pad | OG_A);
    }
    if ((released & OG_START) && !start_chord) frozen = !frozen;
    return k & ~OG_START;
}

static void handle_keys(unsigned k) {
    if (page == PAGE_SPEC) k = glow_keys(scale_keys(k));
    else if (page == PAGE_WIFI || page == PAGE_ZB) k = glow_keys(k);
    if (k & OG_MENU) {
        /* SPEC -> WIFI/ZIGBEE (whichever was last) -> HUNT. */
        page = page == PAGE_SPEC ? survey_page : page == PAGE_HUNT ? PAGE_SPEC : PAGE_HUNT;
        og_tone(0);
        if (page == PAGE_SPEC) spec_retuned();
    }
    if (k & OG_VOLUME) {
        if (page == PAGE_SPEC) {
            marks = (marks + 1) % MARKS;
            spec_layout();
        } else if (page == PAGE_HUNT) {
            sound = !sound;
            og_tone_volume(sound ? 1 : 0);
        } else {
            page = survey_page = page == PAGE_WIFI ? PAGE_ZB : PAGE_WIFI;
        }
    }
    const bool fast = held_long();
    if (k & (OG_UP | OG_DOWN)) {
        int step = fast ? 4 : 1;
        set_gain(manual_gain + (k & OG_UP ? step : -step));
        if (page == PAGE_SPEC) agc = false;
        else auto_gain = false;
    }
    if ((k & OG_SELECT) && page != PAGE_SPEC) {
        auto_gain = !auto_gain;
        if (auto_gain) manual_gain = GAIN_REF;
    }
    switch (page) {
    case PAGE_SPEC: {
        int step = fast ? (int)sv_spans[span].msps / 8 : 1;
        if (k & OG_LEFT) spec_centre -= step;
        if (k & OG_RIGHT) spec_centre += step;
        /* Down to 1842 MHz the ESP32 retunes through its 5/6 LO mode
         * (upstream-verified there); LTE band 1 and the top of band 3 fit. */
        if (spec_centre < FREQ_MIN) spec_centre = FREQ_MIN;
        if (spec_centre > FREQ_MAX) spec_centre = FREQ_MAX;
        if (k & OG_A) {
            span = (span + 1) % SV_SPANS;
            fit_pending = true; /* per-bin noise moves with the bin width */
        }
        if (k & OG_SELECT) agc = !agc;
        if (k & (OG_LEFT | OG_RIGHT | OG_A)) spec_retuned();
        break;
    }
    case PAGE_WIFI:
    case PAGE_ZB: {
        survey_t *v = survey();
        const int last = v->first + v->count - 1;
        if (k & OG_LEFT) v->cursor = v->cursor > v->first ? v->cursor - 1 : last;
        if (k & OG_RIGHT) v->cursor = v->cursor < last ? v->cursor + 1 : v->first;
        if (k & OG_START) survey_reset(v);
        if (k & OG_A) {
            hunt_kind = page == PAGE_WIFI ? TGT_WIFI : TGT_ZB;
            hunt_ch = v->cursor;
            hunt_reset();
            page = PAGE_HUNT;
        }
        break;
    }
    case PAGE_HUNT:
        if (k & (OG_LEFT | OG_RIGHT)) {
            int d = k & OG_RIGHT ? 1 : -1;
            if (hunt_kind == TGT_ZB) hunt_ch = (hunt_ch - 11 + d + 16) % 16 + 11;
            else if (hunt_kind == TGT_WIFI) hunt_ch = (hunt_ch - 1 + d + 13) % 13 + 1;
            else hunt_mhz = hunt_mhz + d < FREQ_MIN ? FREQ_MIN : hunt_mhz + d > FREQ_MAX ? FREQ_MAX : hunt_mhz + d;
            hunt_reset();
        }
        if (k & OG_A) {
            /* Zigbee -> Wi-Fi -> MHz, keeping the nearest frequency. */
            int f = (int)hunt_target(), ch;
            hunt_kind = (hunt_kind + 1) % TGTS;
            if (hunt_kind == TGT_WIFI) ch = (f - 2407 + 2) / 5, hunt_ch = ch < 1 ? 1 : ch > 13 ? 13 : ch;
            else if (hunt_kind == TGT_ZB) ch = (f - 2405 + 2) / 5 + 11, hunt_ch = ch < 11 ? 11 : ch > 26 ? 26 : ch;
            else hunt_mhz = (unsigned)f;
            hunt_reset();
        }
        if (k & OG_B) tone_mode = (tone_mode + 1) % 3;
        if (k & OG_START) hunt_reset();
        break;
    }
}

void board_ui_init(void) {
    og_lcd_init();
    og_io_init();
    og_tone_init();
    og_tone_volume(sound ? 1 : 0);
    wf = calloc(WF_ROWS, W);
    /* Optional: without it SPEC offers LIVE and PEAK only. */
    glow = true;
    for (int c = 0; c < GLOW_CHUNKS; c++) glow &= (glow_chunk[c] = calloc(GLOW_ROWS, W)) != NULL;
    if (!glow) view = VIEW_PEAK;
    bool ok = wf && sv_init() && zb_init();
    sv_waterfall_lut(lut);
    /* The drivers own these pins; GPIO discovery must leave them alone. */
    esp_gpio_reserve(og_board_pins());
    spec_layout();
    survey_reset(&wifi_sv);
    survey_reset(&zb_sv);
    battery_update();
    render(splash_strip, 0, H);
    if (!ok) render(error_strip, 0, 16);
    og_lcd_backlight(80);
    splash_until = esp_timer_get_time() + 4000000;
}

/* OGSTAT? reports the UI's state; OGSET <page> <tone> <kind> <ch> sets HUNT
 * up remotely (the UI resumes five seconds after the last command). */
bool board_ui_command(const char *line) {
    char buf[256];
    int pg, tm, kind, ch;
    if (!strcmp(line, "OGSTAT?")) {
        int n = snprintf(buf, sizeof(buf),
            "OGSTAT page %d tone %d sound %d kind %d ch %d target %u bursts %u ticks %u hz %u "
            "now %.1f env %.1f ref %.1f gain %d auto %d clip %.2f heap %u\n",
            page, tone_mode, sound, hunt_kind, hunt_ch, hunt_target(), hunt_bursts, hunt_ticks,
            og_tone_hz(), hunt_now, hunt_env, hunt_ref, manual_gain, auto_gain, clip_rate,
            (unsigned)esp_get_free_heap_size());
        burst_serial_send(buf, n);
        return true;
    }
    if (!strcmp(line, "OGWF?")) {
        /* Newest row first: snapshots, mean of the row's peaks (dB), mean LUT index. */
        int n = snprintf(buf, sizeof(buf), "OGWF ref %d range %d view %d gain %d agc %d capture %u us prep %u fft %u log %u measure %u us snapshot %u us frame %u us\n",
                         ref, range, view, manual_gain, agc, sv_capture_us, sv_prep_us, sv_fft_us, sv_log_us, sv_measure_us, spec_snap_us, spec_frame_us);
        burst_serial_send(buf, n);
        for (int i = 0; i < WF_ROWS; i++) {
            int r = (wf_head - i + WF_ROWS) % WF_ROWS;
            unsigned lut_sum = 0;
            for (int x = 0; x < W; x++) lut_sum += wf[r * W + x];
            n = snprintf(buf, sizeof(buf), "%d %u %.1f %u\n", i, wf_got[r], wf_db10[r] / 10.0, lut_sum / W);
            burst_serial_send(buf, n);
        }
        burst_serial_send("END\n", 4);
        return true;
    }
    int v, g, a = 0;
    if (sscanf(line, "OGSPEC %d %d %d", &v, &g, &a) >= 2 && v >= 0 && v < VIEWS) {
        page = PAGE_SPEC;
        if (a >= 1 && a <= WF_AVG_MAX) wf_avg = a;
        view = v;
        agc = g < 0;
        if (g >= 0) manual_gain = g;
        splash_until = 0;
        burst_serial_send("OK\n", 3);
        return true;
    }
    if (sscanf(line, "OGSET %d %d %d %d", &pg, &tm, &kind, &ch) == 4 && pg >= 0 && pg < PAGES &&
        tm >= 0 && tm < 3 && kind >= 0 && kind < TGTS) {
        page = pg;
        tone_mode = tm;
        hunt_kind = kind;
        if (kind == TGT_MHZ) hunt_mhz = (unsigned)ch;
        else hunt_ch = ch;
        hunt_reset();
        hunt_ticks = 0;
        splash_until = 0;
        burst_serial_send("OK\n", 3);
        return true;
    }
    return false;
}

bool board_ui_step(bool host_active) {
    int64_t now = esp_timer_get_time();
    if (host_active) {
        if (!host_shown) {
            og_tone(0);
            render(host_strip, 0, 16);
            host_shown = true;
        }
        return false;
    }
    if (host_shown) {
        host_shown = false;
        if (page == PAGE_SPEC) spec_retuned();
    }
    unsigned k = pressed();
    if (splash_until) {
        if (!k) return false;
        splash_until = 0;
        swallow = held_prev; /* the key that closed the splash acts on nothing */
        k = 0;
    }
    if (now >= battery_at) {
        battery_update();
        battery_at = now + 2000000;
    }
    handle_keys(k);
    switch (page) {
    case PAGE_SPEC: spec_frame(); break;
    case PAGE_WIFI:
    case PAGE_ZB: survey_frame(survey()); break;
    default: hunt_frame(); break;
    }
    /* Let the idle task run once per frame (task watchdog). */
    vTaskDelay(1);
    return true;
}
