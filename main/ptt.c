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

static SemaphoreHandle_t s_lock;
static esp_timer_handle_t s_tot_timer;
static bool s_on;

static void set_locked(bool on, const char *reason, tone_id_t tone)
{
    if (on == s_on) {
        return;
    }
    s_on = on;
    if (on) {
        /* audio first, then key the radio */
        audio_io_set_tx(true);
        gpio_set_level(CONFIG_BTPTT_PTT_GPIO, PTT_LEVEL(true));
        if (g_settings.tot_s > 0) {
            esp_timer_start_once(s_tot_timer, (uint64_t)g_settings.tot_s * 1000000ULL);
        }
    } else {
        gpio_set_level(CONFIG_BTPTT_PTT_GPIO, PTT_LEVEL(false));
        audio_io_set_tx(false);
        esp_timer_stop(s_tot_timer);
    }
    tone_play(tone);
    ESP_LOGI(TAG, "PTT %s (%s)", on ? "ON  >>> TX" : "OFF <<< RX", reason);
}

static void tot_cb(void *arg)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_on) {
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
    gpio_set_level(CONFIG_BTPTT_PTT_GPIO, PTT_LEVEL(false));
    const esp_timer_create_args_t args = { .callback = tot_cb, .name = "tot" };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_tot_timer));
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
    set_locked(!s_on, reason, !s_on ? TONE_TX_ON : TONE_TX_OFF);
    xSemaphoreGive(s_lock);
}

bool ptt_get(void)
{
    return s_on;
}
