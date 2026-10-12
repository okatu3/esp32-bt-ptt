#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "audio_io.h"
#include "pcm_ring.h"
#include "settings.h"
#include "tone.h"

#define TAG "audio"

#define SAMPLE_RATE      8000
#define FRAME_SAMPLES    64                 /* 8 ms per codec read/write */
#define RING_SAMPLES     2048               /* 256 ms */
#define RING_KEEP        960                /* trim backlog above 120 ms */

#define ES8388_I2C_ADDR  0x10               /* 7-bit */
#define AC101_I2C_ADDR   0x1A               /* 7-bit, other A1S module variant */
#define ES8388_REG_ADCCONTROL2 0x0A
#define ES8388_ADC_INPUT_LINE2 0x50         /* LIN2/RIN2 = AudioKit LINE IN jack */
#define AUDIOKIT_PA_GPIO 21                 /* speaker amplifier enable */

typedef struct {
    const char *name;
    int sda, scl, mclk, bclk, ws, dout, din;
} board_pins_t;

/* The A1S AudioKit v2.2 exists in two ES8388 wirings. */
static const board_pins_t k_boards[] = {
    { "A1S AudioKit v2.2 ES8388 (I2C 33/32)", 33, 32, 0, 27, 25, 26, 35 },
    { "A1S AudioKit v2.2 ES8388 (I2C 18/23)", 18, 23, 0, 5, 25, 26, 35 },
};

static const board_pins_t *s_board;
static esp_codec_dev_handle_t s_codec;

static int16_t s_to_hs_buf[RING_SAMPLES];
static int16_t s_from_hs_buf[RING_SAMPLES];
static pcm_ring_t s_to_hs;   /* radio -> headset */
static pcm_ring_t s_from_hs; /* headset -> radio */

static volatile bool s_tx;
static volatile int32_t s_rx_gain_q8 = 256;
static volatile int32_t s_tx_gain_q8 = 256;
static volatile int16_t s_peak_radio, s_peak_hs;

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

static i2c_master_bus_handle_t probe_board(void)
{
    for (size_t i = 0; i < sizeof(k_boards) / sizeof(k_boards[0]); i++) {
        const board_pins_t *b = &k_boards[i];
        i2c_master_bus_config_t cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = b->sda,
            .scl_io_num = b->scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        i2c_master_bus_handle_t bus;
        if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) {
            continue;
        }
        if (i2c_master_probe(bus, ES8388_I2C_ADDR, 50) == ESP_OK) {
            s_board = b;
            return bus;
        }
        if (i2c_master_probe(bus, AC101_I2C_ADDR, 50) == ESP_OK) {
            ESP_LOGE(TAG, "AC101 codec found on I2C %d/%d: this board variant is not supported", b->sda, b->scl);
        }
        i2c_del_master_bus(bus);
    }
    return NULL;
}

static void audio_task(void *arg)
{
    int16_t stereo[FRAME_SAMPLES * 2];
    int16_t mono[FRAME_SAMPLES];

    for (;;) {
        /* radio speaker -> headset */
        if (esp_codec_dev_read(s_codec, stereo, sizeof(stereo)) == ESP_CODEC_DEV_OK) {
            int32_t g = s_rx_gain_q8;
            for (int i = 0; i < FRAME_SAMPLES; i++) {
                mono[i] = sat16((stereo[2 * i] * g) >> 8);
            }
            s_peak_radio = peak_of(mono, FRAME_SAMPLES, s_peak_radio);
            pcm_ring_write(&s_to_hs, mono, FRAME_SAMPLES);
            pcm_ring_trim(&s_to_hs, RING_KEEP);
        }

        /* headset microphone -> radio (only while transmitting) */
        size_t got = pcm_ring_read(&s_from_hs, mono, FRAME_SAMPLES);
        if (got < FRAME_SAMPLES) {
            memset(&mono[got], 0, (FRAME_SAMPLES - got) * sizeof(int16_t));
        }
        bool tx = s_tx;
        int32_t g = s_tx_gain_q8;
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            int16_t v = tx ? sat16((mono[i] * g) >> 8) : 0;
            stereo[2 * i] = v;
            stereo[2 * i + 1] = v;
        }
        esp_codec_dev_write(s_codec, stereo, sizeof(stereo));
    }
}

esp_err_t audio_io_init(void)
{
    pcm_ring_init(&s_to_hs, s_to_hs_buf, RING_SAMPLES);
    pcm_ring_init(&s_from_hs, s_from_hs_buf, RING_SAMPLES);

    /* keep the on-board speaker amplifier off */
    gpio_reset_pin(AUDIOKIT_PA_GPIO);
    gpio_set_direction(AUDIOKIT_PA_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(AUDIOKIT_PA_GPIO, 0);

    i2c_master_bus_handle_t i2c = probe_board();
    if (!i2c) {
        ESP_LOGE(TAG, "ES8388 not found on any known AudioKit pin set");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "board: %s", s_board->name);

    i2s_chan_handle_t tx_chan, rx_chan;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = s_board->mclk,
            .bclk = s_board->bclk,
            .ws = s_board->ws,
            .dout = s_board->dout,
            .din = s_board->din,
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .rx_handle = rx_chan, .tx_handle = tx_chan };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = { .port = I2C_NUM_0, .addr = ES8388_CODEC_DEFAULT_ADDR, .bus_handle = i2c };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    es8388_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .master_mode = false,
        .pa_pin = -1,
    };
    const audio_codec_if_t *codec_if = es8388_codec_new(&es_cfg);
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    s_codec = esp_codec_dev_new(&dev_cfg);
    if (!s_codec) {
        ESP_LOGE(TAG, "esp_codec_dev_new failed");
        return ESP_FAIL;
    }
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = SAMPLE_RATE,
        .channel = 2,
        .bits_per_sample = 16,
        .mclk_multiple = 256,
    };
    if (esp_codec_dev_open(s_codec, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "esp_codec_dev_open failed");
        return ESP_FAIL;
    }
    /* ADF's ES8388 driver selects the on-board microphones; use the LINE IN jack instead. */
    esp_codec_dev_write_reg(s_codec, ES8388_REG_ADCCONTROL2, ES8388_ADC_INPUT_LINE2);
    esp_codec_dev_set_in_gain(s_codec, 0.0f);
    audio_io_apply_settings();

    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, configMAX_PRIORITIES - 4, NULL, 1);
    return ESP_OK;
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
    if (s_codec) {
        esp_codec_dev_set_out_vol(s_codec, g_settings.out_vol);
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
    return s_board ? s_board->name : "(none)";
}
