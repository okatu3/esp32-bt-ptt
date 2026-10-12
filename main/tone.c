#include <math.h>
#include "freertos/FreeRTOS.h"
#include "tone.h"

#define SAMPLE_RATE 8000
#define SINE_LEN 256
#define TONE_AMPLITUDE 6000 /* about -15 dBFS */

typedef struct {
    uint16_t freq_hz; /* 0 = silence */
    uint16_t ms;
} tone_seg_t;

static const tone_seg_t k_tx_on[] = { { 1200, 120 }, { 0, 0 } };
static const tone_seg_t k_tx_off[] = { { 700, 70 }, { 0, 60 }, { 700, 70 }, { 0, 0 } };
static const tone_seg_t k_tot[] = { { 500, 300 }, { 0, 150 }, { 500, 300 }, { 0, 150 }, { 500, 300 }, { 0, 0 } };
static const tone_seg_t k_connected[] = { { 800, 100 }, { 1200, 150 }, { 0, 0 } };

static int16_t s_sine[SINE_LEN];
static const tone_seg_t *s_seq;
static uint32_t s_seg_left;  /* samples left in the current segment */
static uint32_t s_phase;     /* 16.16 fixed point index into s_sine */
static uint32_t s_step;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void tone_init(void)
{
    for (int i = 0; i < SINE_LEN; i++) {
        s_sine[i] = (int16_t)(TONE_AMPLITUDE * sinf(2.0f * (float)M_PI * i / SINE_LEN));
    }
}

static void load_segment(void)
{
    if (s_seq->ms == 0) {
        s_seq = NULL;
        return;
    }
    s_seg_left = (uint32_t)s_seq->ms * SAMPLE_RATE / 1000;
    s_step = (uint32_t)(((uint64_t)s_seq->freq_hz * SINE_LEN << 16) / SAMPLE_RATE);
    s_phase = 0;
}

void tone_play(tone_id_t id)
{
    const tone_seg_t *seq;
    switch (id) {
    case TONE_TX_ON: seq = k_tx_on; break;
    case TONE_TX_OFF: seq = k_tx_off; break;
    case TONE_TOT: seq = k_tot; break;
    default: seq = k_connected; break;
    }
    portENTER_CRITICAL(&s_mux);
    s_seq = seq;
    load_segment();
    portEXIT_CRITICAL(&s_mux);
}

void tone_mix(int16_t *pcm, size_t n)
{
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < n && s_seq; i++) {
        if (s_seq->freq_hz) {
            int32_t v = pcm[i] + s_sine[(s_phase >> 16) % SINE_LEN];
            pcm[i] = v > 32767 ? 32767 : (v < -32768 ? -32768 : (int16_t)v);
            s_phase += s_step;
        }
        if (--s_seg_left == 0) {
            s_seq++;
            load_segment();
        }
    }
    portEXIT_CRITICAL(&s_mux);
}
