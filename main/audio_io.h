#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* ES8388 on the A1S AudioKit, 8 kHz 16-bit.
 *   LINE IN (LIN2/RIN2, left channel used)  <- radio speaker output
 *   Headphone out (LOUT1/ROUT1)             -> radio microphone input
 */
esp_err_t audio_io_init(void);

/* PTT state: headset microphone audio reaches the radio only while true. */
void audio_io_set_tx(bool tx);

/* Called from the HFP SCO data callback (8 kHz mono PCM). */
void audio_io_from_headset(const int16_t *pcm, size_t n);
void audio_io_to_headset(int16_t *pcm, size_t n);
void audio_io_reset_headset_path(void);

void audio_io_apply_settings(void); /* gains / volume from g_settings */
void audio_io_get_peaks(int16_t *radio_in, int16_t *headset_in); /* peak since last call */
const char *audio_io_board_name(void);
