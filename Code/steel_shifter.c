#include "steel_shifter.h"
#include <math.h>
#include <string.h>

#define PARK_THRESHOLD   1e-4f    /* |ratio-1| below this counts as "not bending"     */
#define PARK_RATE        0.0015f  /* drift rate while parking (~2.6 cents)            */
#define DC_BLOCK_HZ      5.0f
#define TRACK_HOP        441      /* run the tracker every 441 samples (10 ms @ 44.1k) */
#define TRACK_MIN_CONF   0.6f     /* ignore frames less periodic than this            */
#define TRACK_PEAK_FRAC  0.9f     /* pick first peak >= 90% of the best               */
#define TRACK_SMOOTH     0.35f    /* period smoothing per hop                         */
#define TRACK_SILENCE    1e-6f    /* mean-square below this = silence, hold           */
#define WIN_SLEW         0.02f    /* max window change per sample (samples/sample)    */

/* Tap gain: smoothstep of a triangle. g(p) + g(p + 0.5) == 1 exactly. */
static inline float tap_gain(float p)
{
    float t = (p < 0.5f) ? 2.0f * p : 2.0f - 2.0f * p;
    return t * t * (3.0f - 2.0f * t);
}

/* Sample that is 'delay' samples old (delay >= 1), linear interpolation. */
static inline float read_tap(const Shifter *s, float delay)
{
    float pos = (float)(int32_t)s->w - 1.0f - delay;
    float fl  = floorf(pos);
    float fr  = pos - fl;
    uint32_t i0 = (uint32_t)(int32_t)fl & SHIFTER_BUF_MASK;
    uint32_t i1 = (i0 + 1u) & SHIFTER_BUF_MASK;
    return s->buf[i0] * (1.0f - fr) + s->buf[i1] * fr;
}

void shifter_init(Shifter *s, float f0_hz, float fs_hz)
{
    memset(s, 0, sizeof(*s));
    float T0 = fs_hz / f0_hz;                       /* open-string period      */
    float T0d = T0 / (float)TRACK_DECIM;

    s->min_win = 64.0f;
    s->max_win = (float)(SHIFTER_BUF_LEN - 8u);

    s->period_s   = T0;
    s->win        = 2.0f * T0;
    s->win_target = s->win;
    if (s->win < s->min_win) s->win = s->win_target = s->min_win;
    if (s->win > s->max_win) s->win = s->win_target = s->max_win;

    s->phase = 0.0f;                                /* tap 2 starts at full weight */
    s->ratio = 1.0f;
    s->dc_r  = 1.0f - 6.2831853f * DC_BLOCK_HZ / fs_hz;

    /* Search from 2 octaves above the open pitch down to 15% below it. */
    s->lag_min = (int)floorf(T0d * 0.25f);
    if (s->lag_min < 3) s->lag_min = 3;
    s->lag_max = (int)ceilf(T0d * 1.15f);
    s->frame_n = 2 * s->lag_max;
}

void shifter_stagger(Shifter *s, int index, int count)
{
    if (count < 1) count = 1;
    s->hop_cnt = (TRACK_HOP * index) / count;
}

float shifter_tracked_period(const Shifter *s) { return s->period_s; }
float shifter_parked_latency(const Shifter *s) { return 0.5f * s->win + 1.0f; }

/* ------------------------------------------------------------------ */
/* Pitch tracker: normalized autocorrelation on the decimated history.   */
static void track_update(Shifter *s)
{
    const int N = s->frame_n, Lmin = s->lag_min, Lmax = s->lag_max;
    const int total = N + Lmax + 1;
    float tmp[TRACK_HIST_LEN];
    float cum[TRACK_HIST_LEN + 1];
    float r[TRACK_HIST_LEN];

    if (total > (int)TRACK_HIST_LEN) return;               /* cannot happen with E9 pitches */

    uint32_t start = s->hist_w - (uint32_t)total;
    for (int i = 0; i < total; i++) tmp[i] = s->hist[(start + (uint32_t)i) & TRACK_HIST_MASK];
    cum[0] = 0.0f;
    for (int i = 0; i < total; i++) cum[i + 1] = cum[i] + tmp[i] * tmp[i];

    const int base = total - N;                            /* frame = tmp[base..total-1] */
    float E0 = cum[total] - cum[base];
    if (E0 < (float)N * TRACK_SILENCE) return;             /* silence: keep last period  */

    float rmax = -2.0f;
    for (int L = Lmin - 1; L <= Lmax + 1; L++) {
        float dot = 0.0f;
        for (int i = base; i < total; i++) dot += tmp[i] * tmp[i - L];
        float EL = cum[total - L] - cum[base - L];
        r[L] = dot / sqrtf(E0 * EL + 1e-12f);
        if (L >= Lmin && L <= Lmax && r[L] > rmax) rmax = r[L];
    }
    if (rmax < TRACK_MIN_CONF) return;                     /* not clearly periodic       */

    int pick = -1;
    for (int L = Lmin; L <= Lmax; L++) {
        if (r[L] >= TRACK_PEAK_FRAC * rmax && r[L] >= r[L - 1] && r[L] >= r[L + 1]) { pick = L; break; }
    }
    if (pick < 0) return;

    float a = r[pick - 1], b = r[pick], c = r[pick + 1];
    float den = a - 2.0f * b + c, delta = 0.0f;
    if (fabsf(den) > 1e-9f) delta = 0.5f * (a - c) / den;
    if (delta > 1.0f) delta = 1.0f; else if (delta < -1.0f) delta = -1.0f;

    float T = ((float)pick + delta) * (float)TRACK_DECIM;
    s->period_s += TRACK_SMOOTH * (T - s->period_s);
    s->win_target = 2.0f * s->period_s;
    if (s->win_target < s->min_win) s->win_target = s->min_win;
    if (s->win_target > s->max_win) s->win_target = s->max_win;
}

static inline void track_push(Shifter *s, float y)
{
    s->dec_acc += y;
    if (++s->dec_cnt == TRACK_DECIM) {
        s->hist[s->hist_w & TRACK_HIST_MASK] = s->dec_acc * (1.0f / (float)TRACK_DECIM);
        s->hist_w++;
        s->dec_acc = 0.0f;
        s->dec_cnt = 0;
    }
    if (++s->hop_cnt >= TRACK_HOP) { s->hop_cnt = 0; track_update(s); }
}

/* ------------------------------------------------------------------ */
float shifter_process(Shifter *s, float x, float ratio)
{
    /* Remove the DC bias left over from the 1.25 V mid-rail. */
    float y = x - s->dc_x1 + s->dc_r * s->dc_y1;
    s->dc_x1 = x;
    s->dc_y1 = y;

    track_push(s, y);

    /* Slew the window toward 2 periods of the tracked pitch (gently: a moving
     * window adds a small extra pitch shift proportional to its speed). */
    float dw = s->win_target - s->win;
    if (dw >  WIN_SLEW) dw =  WIN_SLEW;
    if (dw < -WIN_SLEW) dw = -WIN_SLEW;
    s->win += dw;

    s->buf[s->w] = y;
    s->w = (s->w + 1u) & SHIFTER_BUF_MASK;

    float p1 = s->phase;
    float p2 = p1 + 0.5f;
    if (p2 >= 1.0f) p2 -= 1.0f;

    float g1  = tap_gain(p1);
    float g2  = 1.0f - g1;
    float out = g1 * read_tap(s, 1.0f + p1 * s->win)
              + g2 * read_tap(s, 1.0f + p2 * s->win);

    float dev = ratio - 1.0f;
    if (fabsf(dev) < PARK_THRESHOLD) {
        /* Not bending: drift to the nearest point where ONE tap has full weight. */
        float target = floorf(s->phase * 2.0f + 0.5f) * 0.5f;
        float diff   = target - s->phase;
        float step   = PARK_RATE / s->win;
        if (fabsf(diff) <= step) s->phase = (target >= 1.0f) ? 0.0f : target;
        else                     s->phase += (diff > 0.0f) ? step : -step;
    } else {
        s->phase -= dev / s->win;                 /* delay shrinks when ratio > 1 */
    }
    if (s->phase >= 1.0f)     s->phase -= 1.0f;
    else if (s->phase < 0.0f) s->phase += 1.0f;

    return out;
}

void shifter_process_block(Shifter *s, const float *in, float *out,
                           int n, float ratio_target)
{
    float r0 = s->ratio;
    float dr = (ratio_target - r0) / (float)n;
    for (int i = 0; i < n; i++)
        out[i] = shifter_process(s, in[i], r0 + dr * (float)(i + 1));
    s->ratio = ratio_target;
}
