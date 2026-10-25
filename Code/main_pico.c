/*
 * main_pico.c  --  Pico 2 (RP2350) entry point. ***NOT COMPILED OR RUN***
 *
 * The changer and shifter it calls (steel_changer.c, steel_shifter.c) are the
 * parts tested on a PC. Everything marked TODO is hardware-specific and still
 * has to be written and debugged on the real board.
 *
 * Needs: pico_stdlib, hardware_adc (add hardware_spi / hardware_pio / hardware_dma
 * and an I2S library when you write the two TODO drivers).
 */
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "steel_changer.h"
#include "steel_shifter.h"

#define FS        44100.0f
#define BLOCK     32
#define MIX_GAIN  0.25f           /* headroom for 10 strings summed          */
#define CAL_PIN   14              /* calibration button to GND               */

static const float OPEN_HZ[NUM_STRINGS] = {   /* E9: F#4 D#4 G#4 E4 B3 G#3 F#3 E3 D3 B2 */
    369.99f, 311.13f, 415.30f, 329.63f, 246.94f, 207.65f, 185.00f, 164.81f, 146.83f, 123.47f };

static Shifter shifters[NUM_STRINGS];
static Changer changer;

/* ---------------- TODO: hardware drivers --------------------------------- */

/* TODO 1: ADS7953 on SPI0 (GP16-19) using PIO/DMA in Auto-1 mode.
 * Fill in[s][0..BLOCK-1] with samples scaled to -1..+1 for strings 1..10
 * (ADS7953 CH0..CH9; 12-bit, mid-scale 2048 = 1.25 V), and knee_raw[0..3]
 * with the latest raw counts from CH10..CH13. Block until a full block is ready. */
static void hw_adc_read_block(float in[NUM_STRINGS][BLOCK], float knee_raw[4])
{
    for (int s = 0; s < NUM_STRINGS; s++)
        for (int i = 0; i < BLOCK; i++) in[s][i] = 0.0f;
    for (int k = 0; k < 4; k++) knee_raw[k] = 0.0f;
}

/* TODO 2: PCM5102A over I2S (GP10 DIN, GP11 BCK, GP12 LRCK) using PIO/DMA.
 * Queue 'n' mono samples (-1..+1) to both channels at 44.1 kHz. */
static void hw_audio_write_block(const float *mono, int n) { (void)mono; (void)n; }

/* ---------------- controls ------------------------------------------------- */

static void read_controls(float raw[NUM_CONTROLS], const float knee_raw[4])
{
    for (int i = 0; i < 3; i++) {                  /* pedals A, B, C on GP26..GP28 */
        adc_select_input(i);
        raw[CTRL_PEDAL_A + i] = (float)adc_read();  /* 0..4095 */
    }
    for (int k = 0; k < 4; k++) raw[CTRL_KNEE_RAISE_FG + k] = knee_raw[k];
}

/* Hold the button, press each control fully for 5 s, release the button.
 * Rest = value at the start; full = the extreme that moved furthest. */
static void calibrate(void)
{
    float in[NUM_STRINGS][BLOCK], knee[4], rest[NUM_CONTROLS], lo[NUM_CONTROLS], hi[NUM_CONTROLS], now[NUM_CONTROLS];
    hw_adc_read_block(in, knee);
    read_controls(rest, knee);
    for (int i = 0; i < NUM_CONTROLS; i++) lo[i] = hi[i] = rest[i];
    absolute_time_t end = make_timeout_time_ms(5000);
    while (absolute_time_diff_us(get_absolute_time(), end) > 0) {
        hw_adc_read_block(in, knee);
        read_controls(now, knee);
        for (int i = 0; i < NUM_CONTROLS; i++) {
            if (now[i] < lo[i]) lo[i] = now[i];
            if (now[i] > hi[i]) hi[i] = now[i];
        }
    }
    for (int i = 0; i < NUM_CONTROLS; i++) {
        float full = (fabsf(hi[i] - rest[i]) > fabsf(lo[i] - rest[i])) ? hi[i] : lo[i];
        if (fabsf(full - rest[i]) > 100.0f)         /* ignore controls that never moved */
            changer_set_calibration(&changer, i, rest[i], full);
    }
    /* TODO: save changer.cal[] to flash so calibration survives power-off. */
}

/* ---------------- main ----------------------------------------------------- */

int main(void)
{
    stdio_init_all();
    adc_init();
    for (int i = 0; i < 3; i++) adc_gpio_init(26 + i);
    gpio_init(CAL_PIN); gpio_set_dir(CAL_PIN, GPIO_IN); gpio_pull_up(CAL_PIN);

    changer_init(&changer);
    for (int s = 0; s < NUM_STRINGS; s++) {
        shifter_init(&shifters[s], OPEN_HZ[s], FS);
        shifter_stagger(&shifters[s], s, NUM_STRINGS);
    }

    float in[NUM_STRINGS][BLOCK], knee[4], raw[NUM_CONTROLS];
    float ratios[NUM_STRINGS], tmp[BLOCK], mix[BLOCK];
    const float dt = (float)BLOCK / FS;

    for (;;) {
        hw_adc_read_block(in, knee);
        if (!gpio_get(CAL_PIN)) calibrate();

        read_controls(raw, knee);
        changer_update(&changer, raw, dt);
        changer_ratios(&changer, ratios);

        for (int i = 0; i < BLOCK; i++) mix[i] = 0.0f;
        for (int s = 0; s < NUM_STRINGS; s++) {
            shifter_process_block(&shifters[s], in[s], tmp, BLOCK, ratios[s]);
            for (int i = 0; i < BLOCK; i++) mix[i] += tmp[i];
        }
        for (int i = 0; i < BLOCK; i++) {
            float v = mix[i] * MIX_GAIN;
            mix[i] = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
        }
        hw_audio_write_block(mix, BLOCK);
    }
}
