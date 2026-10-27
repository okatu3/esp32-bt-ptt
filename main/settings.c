#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "settings.h"

#define TAG "settings"
#define NVS_NS "btptt"
#define NVS_KEY "cfg"
#define NVS_KEY_NAME "name"
#define SETTINGS_VER 1

typedef struct {
    uint8_t ver;
    settings_t s;
} settings_blob_t;

settings_t g_settings;
char g_dev_name[SETTINGS_NAME_MAX];

static void settings_defaults(void)
{
    memset(&g_settings, 0, sizeof(g_settings));
    g_settings.mode = HS_MODE_CALL;
    g_settings.tot_s = CONFIG_BTPTT_TOT_DEFAULT_S;
    g_settings.rx_gain_db = 0;
    g_settings.tx_gain_db = 0;
    g_settings.out_vol = 70;
    strlcpy(g_dev_name, CONFIG_BTPTT_DEVICE_NAME, sizeof(g_dev_name));
}

void settings_load(void)
{
    settings_defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings, using defaults");
        return;
    }
    settings_blob_t blob;
    size_t len = sizeof(blob);
    if (nvs_get_blob(h, NVS_KEY, &blob, &len) == ESP_OK && len == sizeof(blob) && blob.ver == SETTINGS_VER) {
        g_settings = blob.s;
    } else {
        ESP_LOGW(TAG, "saved settings invalid, using defaults");
    }
    len = sizeof(g_dev_name);
    if (nvs_get_str(h, NVS_KEY_NAME, g_dev_name, &len) != ESP_OK) {
        strlcpy(g_dev_name, CONFIG_BTPTT_DEVICE_NAME, sizeof(g_dev_name));
    }
    nvs_close(h);
}

void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return;
    }
    settings_blob_t blob = { .ver = SETTINGS_VER, .s = g_settings };
    esp_err_t err = nvs_set_blob(h, NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    }
}

void settings_set_name(const char *name)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return;
    }
    esp_err_t err;
    if (name && name[0]) {
        strlcpy(g_dev_name, name, sizeof(g_dev_name));
        err = nvs_set_str(h, NVS_KEY_NAME, g_dev_name);
    } else {
        strlcpy(g_dev_name, CONFIG_BTPTT_DEVICE_NAME, sizeof(g_dev_name));
        err = nvs_erase_key(h, NVS_KEY_NAME);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    }
}
