#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "audio_hw.h"
#include "audio_io.h"
#include "pcm_ring.h"
#include "settings.h"
#include "tone.h"

#define RING_SAMPLES 2048 /* 256 ms */
#define RING_KEEP    960  /* trim backlog above 120 ms */

static int16_t s_to_hs_buf[RING_SAMPLES];
static int16_t s_from_hs_buf[RING_SAMPLES];
static pcm_ring_t s_to_hs;   /* radio -> headset */
static pcm_ring_t s_from_hs; /* headset -> radio */

static volatile bool s_tx;
static volatile int32_t s_rx_gain_q8 = 256;
static volatile int32_t s_tx_gain_q8 = 256;
static volatile int16_t s_peak_radio, s_peak_hs;
static bool s_hw_ok;

static int16_t sat16(int32_t v)
{
    return v > 32767 ? 32767 : (v < -32768 ? -32768 : (int16_t)v);
}

static int32_t db_to_q8(int db)
{
    return (int32_t)lroundf(256.0f * powf(10.0f, db / 20.0f));
}

static int16_t peak_of(const int16_t *p, size_t n, int16_t prev)
{
    int16_t pk = prev;
    for (size_t i = 0; i < n; i++) {
        int16_t a = p[i] == -32768 ? 32767 : (int16_t)abs(p[i]);
        if (a > pk) {
            pk = a;
        }
    }
    return pk;
}

esp_err_t audio_io_init(void)
{
    pcm_ring_init(&s_to_hs, s_to_hs_buf, RING_SAMPLES);
    pcm_ring_init(&s_from_hs, s_from_hs_buf, RING_SAMPLES);
    esp_err_t err = audio_hw_init();
    s_hw_ok = (err == ESP_OK);
    audio_io_apply_settings();
    return err;
}

void audio_io_radio_in(const int16_t *pcm, size_t n)
{
    int16_t buf[128];
    int32_t g = s_rx_gain_q8;
    while (n > 0) {
        size_t m = n < 128 ? n : 128;
        for (size_t i = 0; i < m; i++) {
            buf[i] = sat16((pcm[i] * g) >> 8);
        }
        s_peak_radio = peak_of(buf, m, s_peak_radio);
        pcm_ring_write(&s_to_hs, buf, m);
        pcm += m;
        n -= m;
    }
    pcm_ring_trim(&s_to_hs, RING_KEEP);
}

void audio_io_radio_out(int16_t *pcm, size_t n)
{
    size_t got = pcm_ring_read(&s_from_hs, pcm, n);
    if (got < n) {
        memset(&pcm[got], 0, (n - got) * sizeof(int16_t));
    }
    bool tx = s_tx;
    int32_t g = s_tx_gain_q8;
    for (size_t i = 0; i < n; i++) {
        pcm[i] = tx ? sat16((pcm[i] * g) >> 8) : 0;
    }
}

void audio_io_set_tx(bool tx)
{
    if (tx && !s_tx) {
        pcm_ring_clear(&s_from_hs); /* start TX with fresh audio */
    }
    s_tx = tx;
}

void audio_io_from_headset(const int16_t *pcm, size_t n)
{
    s_peak_hs = peak_of(pcm, n, s_peak_hs);
    pcm_ring_write(&s_from_hs, pcm, n);
    pcm_ring_trim(&s_from_hs, RING_KEEP);
}

void audio_io_to_headset(int16_t *pcm, size_t n)
{
    size_t got = pcm_ring_read(&s_to_hs, pcm, n);
    if (got < n) {
        memset(&pcm[got], 0, (n - got) * sizeof(int16_t));
    }
    tone_mix(pcm, n);
}

void audio_io_reset_headset_path(void)
{
    pcm_ring_clear(&s_to_hs);
    pcm_ring_clear(&s_from_hs);
}

void audio_io_apply_settings(void)
{
    s_rx_gain_q8 = db_to_q8(g_settings.rx_gain_db);
    s_tx_gain_q8 = db_to_q8(g_settings.tx_gain_db);
    if (s_hw_ok) {
        audio_hw_set_out_vol(g_settings.out_vol);
    }
}

void audio_io_get_peaks(int16_t *radio_in, int16_t *headset_in)
{
    *radio_in = s_peak_radio;
    *headset_in = s_peak_hs;
    s_peak_radio = 0;
    s_peak_hs = 0;
}

const char *audio_io_board_name(void)
{
    return audio_hw_name();
}
