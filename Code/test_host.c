/* Host-side tests for the changer and shifter. Build: make test */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "steel_changer.h"
#include "steel_shifter.h"

#define FS    44100.0
#define BLOCK 32

static const float F0[NUM_STRINGS] = {   /* E9: F#4 D#4 G#4 E4 B3 G#3 F#3 E3 D3 B2 */
    369.99f, 311.13f, 415.30f, 329.63f, 246.94f, 207.65f, 185.00f, 164.81f, 146.83f, 123.47f };

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* Peak frequency of x relative to f_expected, in cents (Hann-windowed DFT sweep). */
static double cents_off(const float *x, int n, double f_expected)
{
    enum { NC = 801 };               /* -200..+200 cents in 0.5 cent steps */
    static double mag[NC];
    double *w = malloc(sizeof(double) * n);
    for (int i = 0; i < n; i++) w[i] = 0.5 - 0.5 * cos(2.0 * M_PI * i / (n - 1));
    for (int k = 0; k < NC; k++) {
        double cents = -200.0 + 0.5 * k;
        double f = f_expected * pow(2.0, cents / 1200.0);
        double dph = 2.0 * M_PI * f / FS;
        double cr = cos(dph), ci = sin(dph);
        double pr = 1.0, pi_ = 0.0, sr = 0.0, si = 0.0;
        for (int i = 0; i < n; i++) {
            double v = w[i] * x[i];
            sr += v * pr; si += v * pi_;
            double nr = pr * cr - pi_ * ci;
            pi_ = pr * ci + pi_ * cr; pr = nr;
        }
        mag[k] = sqrt(sr * sr + si * si);
    }
    free(w);
    int best = 0;
    for (int k = 1; k < NC; k++) if (mag[k] > mag[best]) best = k;
    double off = 0.0;
    if (best > 0 && best < NC - 1) {
        double a = log(mag[best - 1] + 1e-12), b = log(mag[best] + 1e-12), c = log(mag[best + 1] + 1e-12);
        double den = a - 2.0 * b + c;
        if (fabs(den) > 1e-12) off = 0.5 * (a - c) / den;
    }
    return -200.0 + 0.5 * (best + off);
}

static double rms(const float *x, int n)
{
    double s = 0; for (int i = 0; i < n; i++) s += (double)x[i] * x[i];
    return sqrt(s / n);
}

/* ---------------------------------------------------------------- */
static void test_changer_logic(void)
{
    printf("[1] changer logic\n");
    Changer c; changer_init(&c);
    float semis[NUM_STRINGS];

    #define SET_ALL_ZERO() memset(c.position, 0, sizeof c.position)

    SET_ALL_ZERO(); c.position[CTRL_PEDAL_A] = 1.0f;
    changer_semitones(&c, semis);
    CHECK(semis[4] == 2.0f && semis[9] == 2.0f && semis[0] == 0.0f, "pedal A: s5=%.2f s10=%.2f s1=%.2f", semis[4], semis[9], semis[0]);

    SET_ALL_ZERO(); c.position[CTRL_PEDAL_A] = 1.0f; c.position[CTRL_PEDAL_C] = 1.0f;
    changer_semitones(&c, semis);
    CHECK(semis[4] == 2.0f, "A+C on string 5 should stay +2, got %.2f", semis[4]);
    CHECK(semis[3] == 2.0f, "A+C: string 4 should be +2, got %.2f", semis[3]);

    SET_ALL_ZERO(); c.position[CTRL_PEDAL_B] = 1.0f; c.position[CTRL_KNEE_RAISE_FG] = 1.0f;
    changer_semitones(&c, semis);
    CHECK(semis[5] == 1.0f, "B + raise F#/G# on string 6 should be +1, got %.2f", semis[5]);

    SET_ALL_ZERO(); c.position[CTRL_KNEE_RAISE_E] = 1.0f; c.position[CTRL_KNEE_LOWER_E] = 1.0f;
    changer_semitones(&c, semis);
    CHECK(semis[3] == 0.0f && semis[7] == 0.0f, "raise-E + lower-E should cancel, got %.2f %.2f", semis[3], semis[7]);

    SET_ALL_ZERO(); c.position[CTRL_PEDAL_A] = 0.5f;
    changer_semitones(&c, semis);
    CHECK(fabsf(semis[4] - 1.0f) < 1e-6f, "half pedal A should be +1.0, got %.3f", semis[4]);

    /* calibration + snapping + slew: feed a noisy rest value, then full travel */
    changer_init(&c);
    changer_set_calibration(&c, CTRL_PEDAL_A, 1000.0f, 3000.0f);
    float raw[NUM_CONTROLS]; for (int i = 0; i < NUM_CONTROLS; i++) raw[i] = 0.0f;
    float dt = (float)(BLOCK / FS);
    raw[CTRL_PEDAL_A] = 1030.0f;                      /* 1.5% noise at rest */
    for (int i = 0; i < 100; i++) changer_update(&c, raw, dt);
    CHECK(c.position[CTRL_PEDAL_A] == 0.0f, "noise at rest should snap to 0, got %f", c.position[CTRL_PEDAL_A]);
    raw[CTRL_PEDAL_A] = 3000.0f;
    int steps_to_full = 0;
    while (c.position[CTRL_PEDAL_A] < 1.0f && steps_to_full < 100000) { changer_update(&c, raw, dt); steps_to_full++; }
    printf("  full travel reached in %.1f ms (slew limit)\n", 1000.0 * steps_to_full * dt);
    CHECK(c.position[CTRL_PEDAL_A] == 1.0f, "should reach exactly 1.0");
    raw[CTRL_PEDAL_A] = 1000.0f;
    for (int i = 0; i < 400; i++) changer_update(&c, raw, dt);
    float ratios[NUM_STRINGS]; changer_ratios(&c, ratios);
    CHECK(ratios[4] == 1.0f, "released pedal should give ratio exactly 1.0, got %.9f", ratios[4]);
}

/* ---------------------------------------------------------------- */
static void test_steady_shift(void)
{
    printf("[2] steady pitch accuracy, every string, +2 +1 -1 -2 semitones\n");
    const float offs[4] = { 2.0f, 1.0f, -1.0f, -2.0f };
    int total = (int)(1.5 * FS), tail = (int)(0.5 * FS);
    float *in = calloc(total, sizeof(float)), *out = calloc(total, sizeof(float));
    double worst = 0.0; int worst_s = 0; float worst_o = 0;
    for (int s = 0; s < NUM_STRINGS; s++) {
        for (int k = 0; k < 4; k++) {
            Shifter *sh = malloc(sizeof(Shifter));
            shifter_init(sh, F0[s], (float)FS);
            for (int i = 0; i < total; i++) in[i] = 0.5f * (float)sin(2.0 * M_PI * F0[s] * i / FS);
            float ratio = exp2f(offs[k] / 12.0f);
            for (int i = 0; i + BLOCK <= total; i += BLOCK) shifter_process_block(sh, in + i, out + i, BLOCK, ratio);
            double expected = F0[s] * pow(2.0, offs[k] / 12.0);
            double err = cents_off(out + total - tail, tail, expected);
            if (fabs(err) > fabs(worst)) { worst = err; worst_s = s + 1; worst_o = offs[k]; }
            CHECK(fabs(err) < 2.0, "string %d offset %+.0f: %.2f cents off", s + 1, offs[k], err);
            free(sh);
        }
    }
    printf("  worst error: %.2f cents (string %d, %+.0f semitones)\n", worst, worst_s, worst_o);
    free(in); free(out);
}

/* ---------------------------------------------------------------- */
static void test_release_parks(void)
{
    printf("[3] bend then release: pitch returns, level returns, no comb filtering\n");
    int s = 4;                                   /* string 5, B3 */
    int n1 = (int)(0.6 * FS), n2 = (int)(3.5 * FS), total = n1 + n2, tail = (int)(0.5 * FS);
    float *in = calloc(total, sizeof(float)), *out = calloc(total, sizeof(float));
    Shifter *sh = malloc(sizeof(Shifter)); shifter_init(sh, F0[s], (float)FS);
    for (int i = 0; i < total; i++) in[i] = 0.5f * (float)sin(2.0 * M_PI * F0[s] * i / FS);
    float r_up = exp2f(2.0f / 12.0f);
    for (int i = 0; i + BLOCK <= total; i += BLOCK)
        shifter_process_block(sh, in + i, out + i, BLOCK, i < n1 ? r_up : 1.0f);
    double err = cents_off(out + total - tail, tail, F0[s]);
    double ratio_db = 20.0 * log10(rms(out + total - tail, tail) / rms(in + total - tail, tail));
    printf("  after release: %.2f cents off, level %.2f dB vs input, phase=%.3f\n", err, ratio_db, sh->phase);
    CHECK(fabs(err) < 2.0, "pitch after release %.2f cents off", err);
    CHECK(fabs(ratio_db) < 0.5, "level after release %.2f dB", ratio_db);
    CHECK(fabs(sh->phase * 2.0f - roundf(sh->phase * 2.0f)) < 1e-3f, "phase not parked: %f", sh->phase);
    free(in); free(out); free(sh);
}

/* ---------------------------------------------------------------- */
static void test_full_chain(void)
{
    printf("[4] full chain: pedal raw ADC ramp -> changer -> shifter\n");
    Changer c; changer_init(&c);
    changer_set_calibration(&c, CTRL_PEDAL_A, 2000.0f, 2900.0f);
    int total = (int)(2.0 * FS), tail = (int)(0.5 * FS);
    int strings[2] = { 4, 9 };
    for (int q = 0; q < 2; q++) {
        int s = strings[q];
        Changer ch = c;
        Shifter *sh = malloc(sizeof(Shifter)); shifter_init(sh, F0[s], (float)FS);
        float *in = calloc(total, sizeof(float)), *out = calloc(total, sizeof(float));
        for (int i = 0; i < total; i++) in[i] = 0.5f * (float)sin(2.0 * M_PI * F0[s] * i / FS);
        float dt = (float)(BLOCK / FS);
        for (int i = 0; i + BLOCK <= total; i += BLOCK) {
            double t = i / FS;
            float raw[NUM_CONTROLS] = {0};
            raw[CTRL_PEDAL_A] = (float)(2000.0 + 900.0 * fmin(1.0, fmax(0.0, (t - 0.3) / 0.15)));  /* 150 ms press */
            changer_update(&ch, raw, dt);
            float ratios[NUM_STRINGS]; changer_ratios(&ch, ratios);
            shifter_process_block(sh, in + i, out + i, BLOCK, ratios[s]);
        }
        double expected = F0[s] * pow(2.0, 2.0 / 12.0);
        double err = cents_off(out + total - tail, tail, expected);
        printf("  string %d with pedal A held: %.2f cents off target\n", s + 1, err);
        CHECK(fabs(err) < 2.0, "string %d: %.2f cents off", s + 1, err);
        free(in); free(out); free(sh);
    }
}

/* ---------------------------------------------------------------- */
static void test_latency(void)
{
    printf("[5] latency when not bending (impulse response)\n");
    for (int s = 0; s < NUM_STRINGS; s += 3) {
        Shifter *sh = malloc(sizeof(Shifter)); shifter_init(sh, F0[s], (float)FS);
        float peak = 0; int at = 0;
        for (int i = 0; i < 4000; i++) {
            float y = shifter_process(sh, i == 100 ? 1.0f : 0.0f, 1.0f);
            if (fabsf(y) > peak) { peak = fabsf(y); at = i - 100; }
        }
        printf("  string %2d (%6.2f Hz): %5.1f samples = %.1f ms (predicted %.1f)\n",
               s + 1, F0[s], (double)at, 1000.0 * at / FS, 1000.0 * shifter_parked_latency(sh) / FS);
        CHECK(fabs(at - shifter_parked_latency(sh)) < 2.0, "latency mismatch");
        free(sh);
    }
}


/* ---------------------------------------------------------------- */
/* Harmonic-rich decaying "pluck" at frequency f (1..6 harmonics). */
static float pluck_sample(double f, double t, double decay)
{
    double v = 0;
    for (int k = 1; k <= 6; k++) v += sin(2.0 * M_PI * f * k * t) / k;
    return (float)(0.25 * v * exp(-t / decay));
}

static void short_rms_range(const float *x, int n, double *mn, double *mx)
{
    int hop = (int)(0.005 * FS);
    *mn = 1e9; *mx = 0;
    for (int i = 0; i + hop <= n; i += hop) {
        double r = rms(x + i, hop);
        if (r < *mn) *mn = r;
        if (r > *mx) *mx = r;
    }
}

static void test_slide_positions(void)
{
    printf("[6] note slid away from open pitch, pedal A (+2 st) held; harmonic-rich input\n");
    printf("  slide(st)  tracked f (Hz)  pitch err (cents)  level dip (dB)\n");
    int s = 4;                                           /* string 5, B3 */
    int total = (int)(2.5 * FS), tail = (int)(0.5 * FS);
    float *in = calloc(total, sizeof(float)), *out = calloc(total, sizeof(float));
    double worst_dip = 0, worst_err = 0;
    for (int st = 0; st <= 12; st += 1) {
        double f = F0[s] * pow(2.0, st / 12.0);
        Shifter *sh = malloc(sizeof(Shifter)); shifter_init(sh, F0[s], (float)FS);
        for (int i = 0; i < total; i++) in[i] = pluck_sample(f, i / FS, 100.0);   /* nearly steady */
        float r = exp2f(2.0f / 12.0f);
        for (int i = 0; i + BLOCK <= total; i += BLOCK) shifter_process_block(sh, in + i, out + i, BLOCK, r);
        double expected = f * pow(2.0, 2.0 / 12.0);
        double err = cents_off(out + total - tail, tail, expected);
        double mn, mx; short_rms_range(out + total - tail, tail, &mn, &mx);
        double dip = 20.0 * log10(mn / mx);
        double tracked = FS / shifter_tracked_period(sh);
        printf("   %2d        %7.2f          %6.2f           %6.2f\n", st, tracked, err, dip);
        if (fabs(dip) > fabs(worst_dip)) worst_dip = dip;
        if (fabs(err) > fabs(worst_err)) worst_err = err;
        CHECK(fabs(err) < 3.0, "slide %d st: pitch %.2f cents off", st, err);
        CHECK(fabs(dip) < 3.0, "slide %d st: level dip %.2f dB", st, dip);
        free(sh);
    }
    printf("  worst pitch error %.2f cents, worst level dip %.2f dB\n", worst_err, worst_dip);
    free(in); free(out);
}

static void test_glide(void)
{
    printf("[7] continuous glide: open pitch up 12 semitones in 0.6 s, then pedal A pressed\n");
    int s = 4;
    int total = (int)(3.0 * FS);
    float *in = calloc(total, sizeof(float)), *out = calloc(total, sizeof(float));
    Shifter *sh = malloc(sizeof(Shifter)); shifter_init(sh, F0[s], (float)FS);
    double ph = 0;
    for (int i = 0; i < total; i++) {
        double t = i / FS;
        double st = 12.0 * fmin(1.0, fmax(0.0, (t - 0.5) / 0.6));      /* glide 0.5..1.1 s */
        double f = F0[s] * pow(2.0, st / 12.0);
        ph += 2.0 * M_PI * f / FS;
        double v = 0; for (int k = 1; k <= 6; k++) v += sin(k * ph) / k;
        in[i] = (float)(0.25 * v);
    }
    float r = exp2f(2.0f / 12.0f);
    for (int i = 0; i + BLOCK <= total; i += BLOCK) shifter_process_block(sh, in + i, out + i, BLOCK, i / FS > 0.2 ? r : 1.0f);
    double mn, mx; short_rms_range(out + (int)(0.5 * FS), (int)(1.0 * FS), &mn, &mx);
    double dip = 20.0 * log10(mn / mx);
    double final_f = F0[s] * 2.0 * pow(2.0, 2.0 / 12.0);
    double err = cents_off(out + total - (int)(0.5 * FS), (int)(0.5 * FS), final_f);
    printf("  during glide: worst level dip %.2f dB; after glide: %.2f cents off\n", dip, err);
    CHECK(fabs(dip) < 6.0, "glide level dip %.2f dB", dip);
    CHECK(fabs(err) < 3.0, "after glide %.2f cents off", err);
    free(in); free(out); free(sh);
}

int main(void)
{
    test_changer_logic();
    test_steady_shift();
    test_release_parks();
    test_full_chain();
    test_latency();
    test_slide_positions();
    test_glide();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
