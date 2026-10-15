/* Burst-aware band power (see odroidgo.h). One capture is cut into segments
 * of ZB_FFT_N samples; each segment is windowed and transformed, and the
 * power inside every band is kept per segment. A 2 MHz Zigbee frame lasting
 * only part of the capture still stands out in the band's peak over time,
 * while the capture average would dilute it. */
#include <math.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include "odroidgo.h"
#include "dsps_fft2r.h"

#define N ZB_FFT_N
#define MAX_SEGMENTS 64
#define SMOOTH 4           /* segments in the peak's moving average */
#define USABLE 0.42f       /* fraction of the sample rate each side of the LO */
#define SPUR_MHZ 0.25f     /* half width excluded around DC and crystal harmonics */
#define CLIP_LEVEL 500     /* of 511 */

static float *buf, *win, *avg, *seg, *scratch;
static float norm;

bool zb_init(void) {
    if (buf) return true;
    buf = memalign(16, 2 * N * sizeof(float));
    win = malloc(N * sizeof(float));
    avg = malloc(N * sizeof(float));
    scratch = malloc(N * sizeof(float));
    seg = malloc(ZB_MAX_BANDS * MAX_SEGMENTS * sizeof(float));
    if (!(buf && win && avg && scratch && seg)) return false;
    if (dsps_fft2r_init_fc32(NULL, N) != ESP_OK) return false;
    float sum = 0;
    for (int n = 0; n < N; n++) {
        win[n] = 0.5f - 0.5f * cosf(2 * (float)M_PI * n / N);
        sum += win[n];
    }
    /* A full-scale (10-bit) tone reads 0 dB. */
    norm = 1.0f / ((512.0f * sum) * (512.0f * sum));
    return true;
}

static inline int s10(uint32_t v) { return (int)(v << 22) >> 22; }

static float median(const float *v, int n) {
    memcpy(scratch, v, n * sizeof(float));
    for (int i = 1; i < n; i++) {
        float x = scratch[i];
        int j = i - 1;
        for (; j >= 0 && scratch[j] > x; j--) scratch[j + 1] = scratch[j];
        scratch[j + 1] = x;
    }
    return scratch[n / 2];
}

static float db(float p) { return 10 * log10f(p * norm + 1e-20f); }

bool zb_analyze(const uint32_t *iq, unsigned n, unsigned msps, unsigned lo_mhz,
                const float *band_mhz, int bands, float half_mhz, float busy_db,
                zb_result_t *out) {
    if (!buf || bands > ZB_MAX_BANDS) return false;
    int segments = n / N;
    if (segments > MAX_SEGMENTS) segments = MAX_SEGMENTS;
    if (segments < SMOOTH) return false;
    const float bin_mhz = (float)msps / N, usable = USABLE * msps;

    /* Shifted bin index k (0..N-1) sits at (k - N/2) * bin_mhz from the LO.
     * Spurs: the LO's own DC leak and the 40 MHz crystal's harmonics. */
    static uint8_t spur[N];
    for (int k = 0; k < N; k++) {
        float off = (k - N / 2) * bin_mhz, rf = lo_mhz + off;
        float h = fabsf(rf - 40.0f * roundf(rf / 40.0f));
        spur[k] = fabsf(off) > usable || fabsf(off) < SPUR_MHZ + bin_mhz / 2 || h < SPUR_MHZ + bin_mhz / 2;
    }
    int first[ZB_MAX_BANDS], last[ZB_MAX_BANDS];
    for (int b = 0; b < bands; b++) {
        float off = band_mhz[b] - lo_mhz;
        first[b] = (int)ceilf((off - half_mhz) / bin_mhz) + N / 2;
        last[b] = (int)floorf((off + half_mhz) / bin_mhz) + N / 2;
        if (fabsf(off) + half_mhz > usable || first[b] < 0 || last[b] >= N) first[b] = last[b] = -1;
    }

    int64_t si = 0, sq = 0;
    unsigned clipped = 0;
    int peak = 0;
    const unsigned used = (unsigned)segments * N;
    for (unsigned j = 0; j < used; j++) {
        int i = s10(iq[j]), q = s10(iq[j] >> 10);
        si += i; sq += q;
        int a = i < 0 ? -i : i, b = q < 0 ? -q : q;
        if (a > peak) peak = a;
        if (b > peak) peak = b;
        if (i >= CLIP_LEVEL || i <= -CLIP_LEVEL || q >= CLIP_LEVEL || q <= -CLIP_LEVEL) clipped++;
    }
    const float mi = (float)si / used, mq = (float)sq / used;
    out->clip = clipped * 1000 > used;
    out->peak_abs = peak;

    memset(avg, 0, N * sizeof(float));
    for (int s = 0; s < segments; s++) {
        const uint32_t *p = iq + s * N;
        for (int j = 0; j < N; j++) {
            /* Conjugate: RF above the LO arrives at negative frequency. */
            buf[2 * j] = (s10(p[j]) - mi) * win[j];
            buf[2 * j + 1] = (mq - s10(p[j] >> 10)) * win[j];
        }
        dsps_fft2r_fc32(buf, N);
        dsps_bit_rev_fc32(buf, N);
        for (int j = 0; j < N; j++) {
            int k = (j + N / 2) & (N - 1); /* fftshift */
            avg[k] += buf[2 * j] * buf[2 * j] + buf[2 * j + 1] * buf[2 * j + 1];
        }
        for (int b = 0; b < bands; b++) {
            float sum = 0;
            int count = 0;
            for (int k = first[b]; k >= 0 && k <= last[b]; k++) {
                if (spur[k]) continue;
                int j = (k + N / 2) & (N - 1);
                sum += buf[2 * j] * buf[2 * j] + buf[2 * j + 1] * buf[2 * j + 1];
                count++;
            }
            seg[b * MAX_SEGMENTS + s] = count ? sum / count : 0;
        }
    }

    int m = 0;
    for (int k = 0; k < N; k++) if (!spur[k]) scratch[m++] = avg[k] / segments;
    out->floor = m ? db(median(scratch, m)) : -200;
    /* Linear power of the busy threshold, per smoothing window. */
    const float busy_run = busy_db < -150 ? INFINITY : powf(10, busy_db / 10) / norm * SMOOTH;
    for (int b = 0; b < bands; b++) {
        out->busy[b] = 0;
        if (first[b] < 0) {out->peak[b] = out->mean[b] = -200; continue;}
        const float *v = seg + b * MAX_SEGMENTS;
        float run = 0, best = 0, total = 0;
        int windows = 0, busy = 0;
        for (int s = 0; s < segments; s++) {
            run += v[s];
            total += v[s];
            if (s >= SMOOTH) run -= v[s - SMOOTH];
            if (s < SMOOTH - 1) continue;
            if (run > best) best = run;
            windows++;
            busy += run > busy_run;
        }
        out->peak[b] = db(best / SMOOTH);
        out->mean[b] = db(total / segments);
        out->busy[b] = (float)busy / windows;
    }
    return true;
}
