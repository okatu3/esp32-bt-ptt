#pragma once

#include <stddef.h>
#include <stdint.h>

/* Short beeps mixed into the audio sent to the headset (8 kHz). */
typedef enum {
    TONE_TX_ON,     /* one high beep */
    TONE_TX_OFF,    /* two low beeps */
    TONE_TOT,       /* three long low beeps: TX time-out */
    TONE_CONNECTED, /* rising two-tone */
} tone_id_t;

void tone_init(void);
void tone_play(tone_id_t id);
void tone_mix(int16_t *pcm, size_t n);
