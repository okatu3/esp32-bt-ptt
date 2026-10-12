/*
 * BT-PTT: ESP32 (A1S AudioKit v2.2) as Bluetooth HFP Audio Gateway between a
 * Bluetooth headset and a radio (ICOM ID-31).
 *
 *   headset --HFP/SCO (CVSD 8 kHz)--> ESP32 --ES8388--> radio MIC / SP
 *   headset MFB press (AT+CHUP / ATA / AT+BLDN / AT+BVRA / AT+CKPD) -> PTT toggle
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_io.h"
#include "bt_ag.h"
#include "console.h"
#include "ptt.h"
#include "settings.h"
#include "tone.h"

#define TAG "main"

#define KEY_POLL_MS       20
#define KEY_LONG_PRESS_MS 3000

/* Local key: short press toggles PTT, 3 s hold searches for a new headset. */
static void key_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_BTPTT_KEY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = CONFIG_BTPTT_KEY_GPIO < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io);

    int stable = 1, last = 1, same = 0;
    int64_t pressed_at = 0;
    bool long_done = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(KEY_POLL_MS));
        int lvl = gpio_get_level(CONFIG_BTPTT_KEY_GPIO);
        same = (lvl == last) ? same + 1 : 0;
        last = lvl;
        if (same == 2 && lvl != stable) {
            stable = lvl;
            if (stable == 0) {
                pressed_at = esp_timer_get_time();
                long_done = false;
            } else if (!long_done) {
                ptt_toggle("local key");
            }
        }
        if (stable == 0 && !long_done && esp_timer_get_time() - pressed_at > KEY_LONG_PRESS_MS * 1000LL) {
            long_done = true;
            ESP_LOGI(TAG, "local key held: searching for a headset");
            bt_ag_scan(true);
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    settings_load();

    tone_init();
    ptt_init(); /* PTT output inactive as early as possible */
    if (audio_io_init() != ESP_OK) {
        ESP_LOGE(TAG, "audio init failed: continuing without codec");
    }
    ESP_ERROR_CHECK(bt_ag_init());
    console_start();
#if CONFIG_BTPTT_KEY_GPIO >= 0
    xTaskCreate(key_task, "key", 3072, NULL, 4, NULL);
#endif
}
