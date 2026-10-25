/*
 * steel_shifter.h
 *
 * Per-string pitch shifter. Each string is a single note, so every string gets:
 *   1. a DC blocker,
 *   2. a small pitch tracker (autocorrelation on a 4x-decimated copy),
 *   3. a dual-tap delay-line shifter whose window is kept at exactly TWO
 *      periods of the note being played.
 *
 * Why track pitch? With a fixed window the two taps fall out of phase whenever
 * the slide moves the note away from the open-string pitch, and the output
 * level pulses (measured: up to -18 dB). Keeping the half-window equal to one
 * period of the played note keeps both taps in phase at every slide position.
 *
 * Pure C: runs identically on a Pico 2 and on a PC.
 */
#ifndef STEEL_SHIFTER_H
#define STEEL_SHIFTER_H

#include <stdint.h>

#define SHIFTER_BUF_LEN  2048u              /* power of two: audio delay line  */
#define SHIFTER_BUF_MASK (SHIFTER_BUF_LEN - 1u)
#define TRACK_DECIM      4                  /* tracker works at fs / 4         */
#define TRACK_HIST_LEN   512u               /* power of two: decimated history */
#define TRACK_HIST_MASK  (TRACK_HIST_LEN - 1u)

typedef struct {
    /* delay-line shifter */
    float    buf[SHIFTER_BUF_LEN];
    uint32_t w;
    float    phase;          /* 0..1, position of tap 1 in the window         */
    float    win;            /* current window length, samples                */
    float    win_target;     /* window we are slewing toward (2 periods)      */
    float    ratio;          /* ratio at the end of the previous block        */

    /* DC blocker */
    float    dc_x1, dc_y1, dc_r;

    /* pitch tracker */
    float    hist[TRACK_HIST_LEN];
    uint32_t hist_w;
    float    dec_acc;
    int      dec_cnt;
    int      hop_cnt;
    int      lag_min, lag_max, frame_n;   /* in decimated samples            */
    float    period_s;                    /* smoothed period, full-rate samples */
    float    min_win, max_win;
} Shifter;

/* f0_hz = the string's OPEN pitch (sets the tracker's search range). */
void  shifter_init(Shifter *s, float f0_hz, float fs_hz);

/* Spread the tracker updates of several shifters across the 10 ms hop so they
 * don't all run in the same audio block. Call once per string after init:
 * shifter_stagger(&sh[k], k, NUM_STRINGS). */
void  shifter_stagger(Shifter *s, int index, int count);

/* One sample. ratio > 1 raises pitch, ratio < 1 lowers it. */
float shifter_process(Shifter *s, float x, float ratio);

/* n samples; the ratio ramps linearly from the previous block's value. */
void  shifter_process_block(Shifter *s, const float *in, float *out,
                            int n, float ratio_target);

/* Debug / test helpers. */
float shifter_tracked_period(const Shifter *s);     /* samples                 */
float shifter_parked_latency(const Shifter *s);     /* samples, ratio == 1     */

#endif
