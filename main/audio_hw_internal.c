/* Audio backend: ESP32-WROOM with the chip's own ADC and DAC (no codec).
 *   GPIO34 (ADC1_CH6) <- radio speaker output (AC coupled, biased to ~1.3 V)
 *   GPIO25 (DAC1)     -> radio microphone input (through an attenuator)
 *
 * On the ESP32, ADC DMA and DAC DMA both need I2S0, so only the ADC uses DMA
 * (24 kHz, decimated to 8 kHz) and the DAC is written from an 8 kHz timer ISR. */
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac_oneshot.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "hal/dac_ll.h"
#include "esp_log.h"
#include "audio_hw.h"

#define TAG "audio"

#define OUT_RATE       8000
#define DECIM          3
#define ADC_RATE       (OUT_RATE * DECIM)
#define ADC_CHANNEL    ADC_CHANNEL_6        /* GPIO34 */
#define DAC_CHANNEL    DAC_CHAN_0           /* GPIO25 */
#define FRAME_SAMPLES  64                   /* 8 ms at 8 kHz */
#define FRAME_BYTES    (FRAME_SAMPLES * DECIM * SOC_ADC_DIGI_RESULT_BYTES)
#define FIR_TAPS       15
#define ADC_TO_PCM     16                   /* 12-bit ADC counts -> 16-bit PCM */
#define DAC_RING       1024                 /* power of 2 */
#define DAC_KEEP       (FRAME_SAMPLES * 4)  /* max queued DAC samples (32 ms) */

static adc_continuous_handle_t s_adc;
static dac_oneshot_handle_t s_dac;
static volatile int32_t s_out_q8 = 256;

/* single producer (audio task) / single consumer (timer ISR), same core */
static uint8_t s_dac_buf[DAC_RING];
static volatile uint32_t s_dac_wr, s_dac_rd;
static uint8_t s_dac_last = 128;

static float s_fir[FIR_TAPS];

static void fir_init(void)
{
    /* windowed-sinc low-pass, 3.4 kHz at 24 kHz, unity DC gain */
    const float fc = 3400.0f / ADC_RATE;
    const int m = FIR_TAPS - 1;
    float sum = 0;
    for (int i = 0; i < FIR_TAPS; i++) {
        float x = i - m / 2.0f;
        float sinc = x == 0 ? 2 * fc : sinf(2 * (float)M_PI * fc * x) / ((float)M_PI * x);
        float w = 0.54f - 0.46f * cosf(2 * (float)M_PI * i / m);
        s_fir[i] = sinc * w;
        sum += s_fir[i];
    }
    for (int i = 0; i < FIR_TAPS; i++) {
        s_fir[i] /= sum;
    }
}

/* runs while flash is busy too (CONFIG_GPTIMER_ISR_CACHE_SAFE): IRAM/DRAM only */
static bool IRAM_ATTR dac_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    uint32_t rd = s_dac_rd;
    if (rd != s_dac_wr) {
        s_dac_last = s_dac_buf[rd & (DAC_RING - 1)];
        s_dac_rd = rd + 1;
    }
    dac_ll_update_output_value(DAC_CHANNEL, s_dac_last);
    return false;
}

static void dac_push(const int16_t *pcm, size_t n)
{
    int32_t q = s_out_q8;
    uint32_t wr = s_dac_wr;
    for (size_t i = 0; i < n; i++) {
        if (wr - s_dac_rd >= DAC_KEEP) {
            break; /* ADC clock slightly fast: drop instead of piling up */
        }
        int32_t v = 128 + ((pcm[i] * q) >> 16);
        s_dac_buf[wr & (DAC_RING - 1)] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
        wr++;
    }
    s_dac_wr = wr;
}

static void audio_task(void *arg)
{
    /* the DAC timer ISR lives on this task's core, next to the producer */
    gptimer_handle_t timer;
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &timer));
    gptimer_alarm_config_t acfg = {
        .alarm_count = 1000000 / OUT_RATE,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &acfg));
    gptimer_event_callbacks_t cbs = { .on_alarm = dac_timer_isr };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    static uint8_t raw[FRAME_BYTES];
    float hist[FIR_TAPS] = { 0 };
    int hpos = 0, phase = 0;
    float dc_x = 0, dc_y = 0;
    int16_t in[FRAME_SAMPLES], out[FRAME_SAMPLES];
    int nin = 0;

    ESP_ERROR_CHECK(adc_continuous_start(s_adc));
    for (;;) {
        uint32_t len = 0;
        if (adc_continuous_read(s_adc, raw, sizeof(raw), &len, ADC_MAX_DELAY) != ESP_OK) {
            continue;
        }
        for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= len; i += SOC_ADC_DIGI_RESULT_BYTES) {
            const adc_digi_output_data_t *d = (const adc_digi_output_data_t *)&raw[i];
            hist[hpos] = (float)d->type1.data;
            hpos = (hpos + 1) % FIR_TAPS;
            if (++phase < DECIM) {
                continue;
            }
            phase = 0;
            float acc = 0;
            for (int k = 0; k < FIR_TAPS; k++) {
                acc += s_fir[k] * hist[(hpos + k) % FIR_TAPS];
            }
            /* remove the bias / DC (one-pole high-pass, ~20 Hz) */
            float y = acc - dc_x + 0.985f * dc_y;
            dc_x = acc;
            dc_y = y;
            float v = y * ADC_TO_PCM;
            in[nin++] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            if (nin == FRAME_SAMPLES) {
                nin = 0;
                audio_io_radio_in(in, FRAME_SAMPLES);
                audio_io_radio_out(out, FRAME_SAMPLES);
                dac_push(out, FRAME_SAMPLES);
            }
        }
    }
}

esp_err_t audio_hw_init(void)
{
    fir_init();

    dac_oneshot_config_t dcfg = { .chan_id = DAC_CHANNEL };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dcfg, &s_dac));
    dac_oneshot_output_voltage(s_dac, 128);

    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = FRAME_BYTES * 4,
        .conv_frame_size = FRAME_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &s_adc));
    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t acfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = ADC_RATE,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc, &acfg));

    ESP_LOGI(TAG, "board: %s", audio_hw_name());
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, configMAX_PRIORITIES - 4, NULL, 1);
    return ESP_OK;
}

void audio_hw_set_out_vol(int vol)
{
    s_out_q8 = vol * 256 / 100;
}

const char *audio_hw_name(void)
{
    return "ESP32-WROOM internal ADC (GPIO34) / DAC (GPIO25)";
}
