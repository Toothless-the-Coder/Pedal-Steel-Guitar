#include "steel_changer.h"
#include <math.h>

/*
 * Standard E9 starting point. VERIFY against an E9 chart before relying on it:
 * real instruments differ. To change what a control does, edit this table only.
 */
const Change CHANGE_TABLE[] = {
    { CTRL_PEDAL_A,        4, +2 }, { CTRL_PEDAL_A,        9, +2 },
    { CTRL_PEDAL_B,        2, +1 }, { CTRL_PEDAL_B,        5, +1 },
    { CTRL_PEDAL_C,        3, +2 }, { CTRL_PEDAL_C,        4, +2 },
    { CTRL_KNEE_RAISE_FG,  0, +1 }, { CTRL_KNEE_RAISE_FG,  5, +1 },
    { CTRL_KNEE_RAISE_E,   3, +1 }, { CTRL_KNEE_RAISE_E,   7, +1 },
    { CTRL_KNEE_LOWER_E,   3, -1 }, { CTRL_KNEE_LOWER_E,   7, -1 },
    { CTRL_KNEE_LOWER_DD,  1, -1 }, { CTRL_KNEE_LOWER_DD,  8, -1 },
};
const int CHANGE_TABLE_LEN = (int)(sizeof(CHANGE_TABLE) / sizeof(CHANGE_TABLE[0]));

static float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

static float move_toward(float cur, float target, float max_step)
{
    float d = target - cur;
    if (fabsf(d) <= max_step) return target;
    return cur + (d > 0.0f ? max_step : -max_step);
}

void changer_init(Changer *c)
{
    for (int i = 0; i < NUM_CONTROLS; i++) {
        c->cal[i].raw_rest = 0.0f;
        c->cal[i].raw_full = 4095.0f;
        c->filt[i] = 0.0f;
        c->position[i] = 0.0f;
    }
    c->tau_s      = 0.003f;  /* 3 ms: removes ADC noise, not audible as lag */
    c->slew_per_s = 20.0f;   /* full travel in at least 50 ms               */
    c->snap       = 0.02f;   /* ends of travel behave like mechanical stops */
}

void changer_set_calibration(Changer *c, int control, float raw_rest, float raw_full)
{
    if (control < 0 || control >= NUM_CONTROLS) return;
    c->cal[control].raw_rest = raw_rest;
    c->cal[control].raw_full = raw_full;
}

void changer_update(Changer *c, const float raw[NUM_CONTROLS], float dt)
{
    float alpha    = 1.0f - expf(-dt / c->tau_s);
    float max_step = c->slew_per_s * dt;

    for (int i = 0; i < NUM_CONTROLS; i++) {
        float span = c->cal[i].raw_full - c->cal[i].raw_rest;
        float t = 0.0f;
        if (fabsf(span) > 1e-3f)
            t = clamp01((raw[i] - c->cal[i].raw_rest) / span);

        if (t < c->snap)               t = 0.0f;
        else if (t > 1.0f - c->snap)   t = 1.0f;

        c->filt[i] += alpha * (t - c->filt[i]);
        if (fabsf(t - c->filt[i]) < 1e-4f) c->filt[i] = t;  /* avoid endless tail */

        c->position[i] = move_toward(c->position[i], c->filt[i], max_step);
    }
}

/*
 * Combining rule per string:
 *   - several controls pushing the same direction: the LARGEST one wins
 *     (A + C on string 5 stays at +2, it does not become +4);
 *   - controls pushing opposite directions: they ADD
 *     (raise-E + lower-E both engaged on string 4 cancel to 0).
 * Check this against how your target instrument behaves.
 */
void changer_semitones(const Changer *c, float out[NUM_STRINGS])
{
    float up[NUM_STRINGS]   = {0};
    float down[NUM_STRINGS] = {0};

    for (int k = 0; k < CHANGE_TABLE_LEN; k++) {
        const Change *e = &CHANGE_TABLE[k];
        float v = c->position[e->control] * (float)e->semitones;
        if (v > up[e->string])   up[e->string]   = v;
        if (v < down[e->string]) down[e->string] = v;
    }
    for (int s = 0; s < NUM_STRINGS; s++) out[s] = up[s] + down[s];
}

void changer_ratios(const Changer *c, float out[NUM_STRINGS])
{
    float semis[NUM_STRINGS];
    changer_semitones(c, semis);
    for (int s = 0; s < NUM_STRINGS; s++) out[s] = exp2f(semis[s] / 12.0f);
}
