#pragma once

/* Board audio backend (one implementation is built, selected in menuconfig).
 * The backend moves 8 kHz 16-bit mono audio between the radio and audio_io.c. */

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Implemented by the backend: audio_hw_es8388.c or audio_hw_internal.c */
esp_err_t audio_hw_init(void);
void audio_hw_set_out_vol(int vol); /* 0-100 */
const char *audio_hw_name(void);

/* Implemented by audio_io.c, called from the backend's audio task */
void audio_io_radio_in(const int16_t *pcm, size_t n); /* radio speaker -> headset */
void audio_io_radio_out(int16_t *pcm, size_t n);      /* headset mic -> radio (silence unless TX) */
