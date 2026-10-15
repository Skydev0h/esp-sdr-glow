/* Shared on-device spectrum measurement and drawing helpers (see sdr_view.h).
 * From fedaykinofdune/CardputerADV-SDR (GPL-3.0); esp-sdr-glow adds the
 * overridable geometry and per-sample gain compensation. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "sdr_view.h"
#include "sdr_local.h"
#include "font5x7.h"

#define BIN0 ((SV_FFT_N - SV_BINS) / 2)

const sv_span_t sv_spans[SV_SPANS] = {{0, 80}, {1, 40}, {6, 16}};

static float *win, *re, *im, *tw_re, *tw_im, *acc, *out;
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
    win = malloc(f); re = malloc(f); im = malloc(f); acc = malloc(f);
    tw_re = malloc(f / 2); tw_im = malloc(f / 2);
    if (!(win && re && im && acc && tw_re && tw_im)) return false;
    out = im; /* free once the FFTs are done; the heap is tight */
    _Static_assert(SV_BINS <= SV_FFT_N, "output fits the FFT buffer");
    float sum = 0;
    for (int n = 0; n < SV_FFT_N; n++) {
        win[n] = 0.5f - 0.5f * cosf(2 * (float)M_PI * n / SV_FFT_N);
        sum += win[n];
    }
    /* Normalise so a full-scale tone reads 0 dBFS (10-bit samples). */
    for (int n = 0; n < SV_FFT_N; n++) win[n] /= sum * 512.0f;
    for (int k = 0; k < SV_FFT_N / 2; k++) {
        tw_re[k] = cosf(2 * (float)M_PI * k / SV_FFT_N);
        tw_im[k] = -sinf(2 * (float)M_PI * k / SV_FFT_N);
    }
#ifdef SV_GAIN_COMP
    for (int g = 0; g < 256; g++)
        gain_k[g] = powf(10, -SV_GAIN_DB_STEP * ((g < SV_GAIN_MIN ? SV_GAIN_MIN : g) - SV_GAIN_REF) / 20);
#endif
    return true;
}

static void fft(void) {
    for (unsigned i = 1, j = 0; i < SV_FFT_N; i++) {
        unsigned bit = SV_FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (unsigned len = 2; len <= SV_FFT_N; len <<= 1) {
        unsigned half = len >> 1, stride = SV_FFT_N / len;
        for (unsigned i = 0; i < SV_FFT_N; i += len)
            for (unsigned k = 0; k < half; k++) {
                float wr = tw_re[k * stride], wi = tw_im[k * stride];
                unsigned a = i + k, b = a + half;
                float tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
            }
    }
}
static inline int s10(uint32_t v) { return (int)(v << 22) >> 22; }

const float *sv_measure(unsigned span, bool dc_fix) {
    const uint32_t *w = sdr_local_capture(SV_FFT_N * SV_SEGMENTS, sv_spans[span].divider);
    if (!w) return NULL;
    memset(acc, 0, SV_FFT_N * sizeof(float));
    for (int s = 0; s < SV_SEGMENTS; s++) {
        const uint32_t *p = w + s * SV_FFT_N;
        /* The LO leaks a DC offset; removing each segment's mean takes it
         * out. Wideband signals at the centre survive, but a carrier within
         * about one bin of the LO is cancelled with it. */
        float si = 0, sq = 0;
        for (int n = 0; n < SV_FFT_N; n++) {
            /* Conjugate: RF above the LO arrives at negative frequency
             * (the web viewer conjugates every target the same way). */
            const float k = GAIN_K(p[n]);
            re[n] = s10(p[n]) * k;
            im[n] = -s10(p[n] >> 10) * k;
            si += re[n];
            sq += im[n];
        }
        const float mi = dc_fix ? si / SV_FFT_N : 0, mq = dc_fix ? sq / SV_FFT_N : 0;
        for (int n = 0; n < SV_FFT_N; n++) {
            re[n] = (re[n] - mi) * win[n];
            im[n] = (im[n] - mq) * win[n];
        }
        fft();
        for (int k = 0; k < SV_FFT_N; k++) acc[k] += re[k] * re[k] + im[k] * im[k];
    }
    for (int x = 0; x < SV_BINS; x++) {
        int k = (x + BIN0 + SV_FFT_N / 2) & (SV_FFT_N - 1); /* fftshift */
        out[x] = 10 * log10f(acc[k] / SV_SEGMENTS + 1e-12f);
    }
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
