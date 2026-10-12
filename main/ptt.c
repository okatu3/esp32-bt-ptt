#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "audio_io.h"
#include "ptt.h"
#include "settings.h"
#include "tone.h"

#define TAG "ptt"

#if CONFIG_BTPTT_PTT_ACTIVE_HIGH
#define PTT_LEVEL(on) ((on) ? 1 : 0)
#else
#define PTT_LEVEL(on) ((on) ? 0 : 1)
#endif

#if CONFIG_BTPTT_LED_ACTIVE_LOW
#define LED_LEVEL(on) ((on) ? 0 : 1)
#else
#define LED_LEVEL(on) ((on) ? 1 : 0)
#endif

#define TOT_WARN_US 6000000 /* 3 s of warning beeps + 3 s to press */

static SemaphoreHandle_t s_lock;
static esp_timer_handle_t s_tot_timer;
static esp_timer_handle_t s_warn_timer;
static bool s_on;
static bool s_warning; /* TOT warning running: a press extends instead of toggling */

static void set_outputs(bool on)
{
    gpio_set_level(CONFIG_BTPTT_PTT_GPIO, PTT_LEVEL(on));
#if CONFIG_BTPTT_LED_GPIO >= 0
    gpio_set_level(CONFIG_BTPTT_LED_GPIO, LED_LEVEL(on));
#endif
}

static void start_tot(void)
{
    esp_timer_stop(s_tot_timer);
    if (g_settings.tot_s > 0) {
        esp_timer_start_once(s_tot_timer, (uint64_t)g_settings.tot_s * 1000000ULL);
    }
}

static void set_locked(bool on, const char *reason, tone_id_t tone)
{
    if (on == s_on) {
        return;
    }
    s_on = on;
    s_warning = false;
    esp_timer_stop(s_warn_timer);
    if (on) {
        /* audio first, then key the radio */
        audio_io_set_tx(true);
        set_outputs(true);
        start_tot();
    } else {
        set_outputs(false);
        audio_io_set_tx(false);
        esp_timer_stop(s_tot_timer);
    }
    tone_play(tone);
    ESP_LOGI(TAG, "PTT %s (%s)", on ? "ON  >>> TX" : "OFF <<< RX", reason);
}

/* TOT reached: warn, and give the operator a few seconds to extend. */
static void tot_cb(void *arg)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_on && !s_warning) {
        ESP_LOGW(TAG, "TX time-out warning: press to extend");
        s_warning = true;
        tone_play(TONE_TOT_WARN);
        esp_timer_start_once(s_warn_timer, TOT_WARN_US);
    }
    xSemaphoreGive(s_lock);
}

static void warn_cb(void *arg)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_on && s_warning) {
        ESP_LOGW(TAG, "TX time-out (%u s)", g_settings.tot_s);
        set_locked(false, "TOT", TONE_TOT);
    }
    xSemaphoreGive(s_lock);
}

void ptt_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    gpio_reset_pin(CONFIG_BTPTT_PTT_GPIO);
    gpio_set_direction(CONFIG_BTPTT_PTT_GPIO, GPIO_MODE_OUTPUT);
#if CONFIG_BTPTT_LED_GPIO >= 0
    gpio_reset_pin(CONFIG_BTPTT_LED_GPIO);
    gpio_set_direction(CONFIG_BTPTT_LED_GPIO, GPIO_MODE_OUTPUT);
#endif
    set_outputs(false);
    const esp_timer_create_args_t args = { .callback = tot_cb, .name = "tot" };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_tot_timer));
    const esp_timer_create_args_t wargs = { .callback = warn_cb, .name = "tot_warn" };
    ESP_ERROR_CHECK(esp_timer_create(&wargs, &s_warn_timer));
}

void ptt_set(bool on, const char *reason)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    set_locked(on, reason, on ? TONE_TX_ON : TONE_TX_OFF);
    xSemaphoreGive(s_lock);
}

void ptt_toggle(const char *reason)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_on && s_warning) {
        s_warning = false;
        esp_timer_stop(s_warn_timer);
        start_tot();
        tone_play(TONE_TX_ON);
        ESP_LOGI(TAG, "TX extended by %u s (%s)", g_settings.tot_s, reason);
    } else {
        set_locked(!s_on, reason, !s_on ? TONE_TX_ON : TONE_TX_OFF);
    }
    xSemaphoreGive(s_lock);
}

bool ptt_get(void)
{
    return s_on;
}
