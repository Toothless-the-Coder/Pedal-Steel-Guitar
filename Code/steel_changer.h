/*
 * steel_changer.h
 *
 * The "changer": turns pedal / knee-lever positions into a pitch change
 * (in semitones, then a frequency ratio) for each of the 10 strings.
 * Pure C, no hardware calls, so it runs the same on a Pico 2 and on a PC.
 */
#ifndef STEEL_CHANGER_H
#define STEEL_CHANGER_H

#include <stdint.h>

#define NUM_STRINGS  10
#define NUM_CONTROLS 7

/* Control indices. Keep in the same order as the raw[] array you pass in. */
enum {
    CTRL_PEDAL_A = 0,      /* strings 5 and 10 up a whole step            */
    CTRL_PEDAL_B,          /* strings 3 and 6 up a half step              */
    CTRL_PEDAL_C,          /* strings 4 and 5 up a whole step             */
    CTRL_KNEE_RAISE_FG,    /* strings 1 and 6 up a half step              */
    CTRL_KNEE_RAISE_E,     /* strings 4 and 8 up a half step              */
    CTRL_KNEE_LOWER_E,     /* strings 4 and 8 down a half step            */
    CTRL_KNEE_LOWER_DD     /* strings 2 and 9 down a half step            */
};

/* One row of the changer table. 'string' is 0-based (string 1 = 0). */
typedef struct {
    uint8_t control;
    uint8_t string;
    int8_t  semitones;     /* change when the control is fully engaged    */
} Change;

extern const Change CHANGE_TABLE[];
extern const int    CHANGE_TABLE_LEN;

typedef struct {
    float raw_rest;        /* raw ADC value with the control released     */
    float raw_full;        /* raw ADC value with the control fully down   */
} ControlCal;

typedef struct {
    ControlCal cal[NUM_CONTROLS];
    float filt[NUM_CONTROLS];      /* noise-filtered position, 0..1       */
    float position[NUM_CONTROLS];  /* final slew-limited position, 0..1   */
    float tau_s;                   /* one-pole smoothing time constant    */
    float slew_per_s;              /* max change in position per second   */
    float snap;                    /* snap to 0 or 1 within this distance */
} Changer;

void changer_init(Changer *c);

/* Store calibration for one control (call after the user sweeps it). */
void changer_set_calibration(Changer *c, int control,
                             float raw_rest, float raw_full);

/* Call once per audio block. raw[] = latest raw ADC value for each control,
 * dt = seconds since the previous call (block_size / sample_rate). */
void changer_update(Changer *c, const float raw[NUM_CONTROLS], float dt);

/* Pitch offset for each string, in semitones (fractional while moving). */
void changer_semitones(const Changer *c, float out[NUM_STRINGS]);

/* Same thing as frequency ratios (2^(semitones/12)). */
void changer_ratios(const Changer *c, float out[NUM_STRINGS]);

#endif
