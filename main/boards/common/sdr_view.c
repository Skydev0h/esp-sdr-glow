/* Shared on-device spectrum measurement and drawing helpers (see sdr_view.h).
 * From fedaykinofdune/CardputerADV-SDR (GPL-3.0); esp-sdr-glow adds the
 * overridable geometry and per-sample gain compensation. */
#include <malloc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "dsps_fft2r.h"
#include "esp_timer.h"
#include "sdr_view.h"
#include "sdr_local.h"
#include "font5x7.h"

#define BIN0 ((SV_FFT_N - SV_BINS) / 2)

const sv_span_t sv_spans[SV_SPANS] = {{0, 80}, {1, 40}, {6, 16}};
unsigned sv_capture_us, sv_measure_us, sv_prep_us, sv_fft_us, sv_log_us;

/* cbuf: interleaved re/im for the esp-dsp radix-2 FFT (ae32 assembly on the
 * ESP32, several times faster than plain C), which shares one twiddle table
 * of at least SV_FFT_N points with every other user of it. */
static float *win, *cbuf, *acc, *out;
#ifdef SV_GAIN_COMP
/* Bits 20-27 of every word hold the gain index that sample was taken with;
 * under hardware AGC it changes inside a capture. Each sample is scaled to
 * SV_GAIN_REF, so levels stay comparable whatever the AGC does. */
static float gain_k[256];
#define GAIN_K(word) gain_k[((word) >> 20) & 0xff]
#else
#define GAIN_K(word) 1.0f
#endif

bool sv_init(void) {
    if (win) return true;
    size_t f = SV_FFT_N * sizeof(float);
    win = malloc(f); acc = malloc(f); out = malloc(SV_BINS * sizeof(float));
    cbuf = memalign(16, 2 * f);
    if (!(win && acc && out && cbuf)) return false;
    if (dsps_fft2r_init_fc32(NULL, SV_FFT_N) != ESP_OK) return false;
    _Static_assert(SV_BINS <= SV_FFT_N, "output fits the FFT buffer");
    float sum = 0;
    for (int n = 0; n < SV_FFT_N; n++) {
        win[n] = 0.5f - 0.5f * cosf(2 * (float)M_PI * n / SV_FFT_N);
        sum += win[n];
    }
    /* Normalise so a full-scale tone reads 0 dBFS (10-bit samples). */
    for (int n = 0; n < SV_FFT_N; n++) win[n] /= sum * 512.0f;
#ifdef SV_GAIN_COMP
    for (int g = 0; g < 256; g++)
        gain_k[g] = powf(10, -SV_GAIN_DB_STEP * ((g < SV_GAIN_MIN ? SV_GAIN_MIN : g) - SV_GAIN_REF) / 20);
#endif
    return true;
}

static inline int s10(uint32_t v) { return (int)(v << 22) >> 22; }

/* 10*log10(x) from the float's exponent plus a quadratic in the mantissa:
 * within about 0.02 dB, a fraction of log10f's cost (480 calls a snapshot). */
static inline float fast_db(float x) {
    union { float f; uint32_t i; } v = {x};
    const float e = (float)((int)(v.i >> 23) - 127);
    v.i = (v.i & 0x7fffffu) | 0x3f800000u;
    const float m = v.f;
    return 3.0103f * (e + (-0.34484843f * m + 2.02466578f) * m - 0.67487759f);
}

const float *sv_measure(unsigned span, bool dc_fix) {
    const int64_t t0 = esp_timer_get_time();
    const uint32_t *w = sdr_local_capture(SV_FFT_N * SV_SEGMENTS, sv_spans[span].divider);
    sv_capture_us = (unsigned)(esp_timer_get_time() - t0);
    if (!w) return NULL;
    memset(acc, 0, SV_FFT_N * sizeof(float));
    int64_t prep = 0, fft = 0, ta;
    for (int s = 0; s < SV_SEGMENTS; s++) {
        ta = esp_timer_get_time();
        const uint32_t *p = w + s * SV_FFT_N;
        /* The LO leaks a DC offset; removing each segment's mean takes it
         * out. Wideband signals at the centre survive, but a carrier within
         * about one bin of the LO is cancelled with it. */
        float si = 0, sq = 0;
        for (int n = 0; n < SV_FFT_N; n++) {
            /* Conjugate: RF above the LO arrives at negative frequency
             * (the web viewer conjugates every target the same way). */
            const float k = GAIN_K(p[n]);
            const float i = s10(p[n]) * k, q = -s10(p[n] >> 10) * k;
            cbuf[2 * n] = i;
            cbuf[2 * n + 1] = q;
            si += i;
            sq += q;
        }
        const float mi = dc_fix ? si / SV_FFT_N : 0, mq = dc_fix ? sq / SV_FFT_N : 0;
        for (int n = 0; n < SV_FFT_N; n++) {
            cbuf[2 * n] = (cbuf[2 * n] - mi) * win[n];
            cbuf[2 * n + 1] = (cbuf[2 * n + 1] - mq) * win[n];
        }
        const int64_t tb = esp_timer_get_time();
        prep += tb - ta;
        dsps_fft2r_fc32(cbuf, SV_FFT_N);
        dsps_bit_rev_fc32(cbuf, SV_FFT_N);
        fft += esp_timer_get_time() - tb;
        for (int k = 0; k < SV_FFT_N; k++) acc[k] += cbuf[2 * k] * cbuf[2 * k] + cbuf[2 * k + 1] * cbuf[2 * k + 1];
    }
    ta = esp_timer_get_time();
    sv_prep_us = (unsigned)prep;
    sv_fft_us = (unsigned)fft;
    for (int x = 0; x < SV_BINS; x++) {
        int k = (x + BIN0 + SV_FFT_N / 2) & (SV_FFT_N - 1); /* fftshift */
        out[x] = fast_db(acc[k] / SV_SEGMENTS + 1e-12f);
    }
    sv_log_us = (unsigned)(esp_timer_get_time() - ta);
    sv_measure_us = (unsigned)(esp_timer_get_time() - t0);
    return out;
}

float sv_bin_mhz(unsigned centre_mhz, unsigned span, float bin) {
    return centre_mhz + (bin + BIN0 - SV_FFT_N / 2) * (float)sv_spans[span].msps / SV_FFT_N;
}
float sv_mhz_bin(unsigned centre_mhz, unsigned span, float mhz) {
    return (mhz - centre_mhz) * SV_FFT_N / (float)sv_spans[span].msps + SV_FFT_N / 2 - BIN0;
}

void sv_resample_max(const float *db, int n, float *dst, int w) {
    for (int x = 0; x < w; x++) {
        int a = x * n / w, b = (x + 1) * n / w;
        if (b <= a) b = a + 1;
        float m = db[a];
        for (int i = a + 1; i < b; i++) if (db[i] > m) m = db[i];
        dst[x] = m;
    }
}

float sv_floor(const float *db, int n, float *s) {
    memcpy(s, db, n * sizeof(float));
    /* Median via partial selection. */
    for (int i = 0; i <= n / 2; i++)
        for (int j = i + 1; j < n; j++)
            if (s[j] < s[i]) {float t = s[i]; s[i] = s[j]; s[j] = t;}
    return s[n / 2];
}

void sv_autoscale(const float *db, int n, float *scratch, int8_t *ref_out, int8_t *range_out) {
    float lo = sv_floor(db, n, scratch), hi = -200;
    for (int x = 0; x < n; x++) if (db[x] > hi) hi = db[x];
    int ref = (int)ceilf((hi + 6) / 5) * 5, bottom = (int)floorf((lo - 8) / 5) * 5;
    if (ref - bottom < 30) bottom = ref - 30;
    if (ref > 10) ref = 10;
    if (ref < -110) ref = -110;
    *ref_out = (int8_t)ref;
    *range_out = (int8_t)(ref - bottom > 100 ? 100 : ref - bottom);
}

void sv_waterfall_lut(uint16_t lut[256]) {
    static const uint8_t stops[][3] = {{0, 0, 0}, {0, 0, 140}, {0, 170, 220}, {240, 230, 0}, {255, 40, 0}, {255, 255, 255}};
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f * 5;
        int s = (int)t;
        if (s > 4) s = 4;
        float f = t - s;
        lut[i] = sv_rgb((unsigned)(stops[s][0] + (stops[s + 1][0] - stops[s][0]) * f),
                        (unsigned)(stops[s][1] + (stops[s + 1][1] - stops[s][1]) * f),
                        (unsigned)(stops[s][2] + (stops[s + 1][2] - stops[s][2]) * f));
    }
}

void sv_fill(uint16_t *buf, int w, int h, uint16_t c) {
    for (int i = 0; i < w * h; i++) buf[i] = c;
}

void sv_text(uint16_t *buf, int w, int h, int x, int y, const char *s, uint16_t fg, int scale) {
    for (; *s; s++, x += 6 * scale) {
        unsigned ch = (unsigned char)*s;
        if (ch < 0x20 || ch > 0x7e) ch = '?';
        if (y + 8 * scale <= 0 || y >= h) continue;
        for (int cx = 0; cx < 5 * scale; cx++) {
            uint8_t col = sv_font5x7[ch - 0x20][cx / scale];
            for (int cy = 0; cy < 8 * scale; cy++) {
                int px = x + cx, py = y + cy;
                if ((col >> (cy / scale)) & 1 && px >= 0 && px < w && py >= 0 && py < h) buf[py * w + px] = fg;
            }
        }
    }
}
