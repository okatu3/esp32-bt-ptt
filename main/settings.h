#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HS_MODE_CALL = 0, /* pretend an active call: MFB sends AT+CHUP */
    HS_MODE_IDLE = 1, /* no call: MFB sends AT+BLDN / AT+BVRA */
} hs_mode_t;

typedef struct {
    bool     has_peer;
    uint8_t  peer[6];     /* headset BD address */
    uint8_t  mode;        /* hs_mode_t */
    uint16_t tot_s;       /* TX time-out, 0 = off */
    int8_t   rx_gain_db;  /* radio -> headset digital gain */
    int8_t   tx_gain_db;  /* headset -> radio digital gain */
    uint8_t  out_vol;     /* ES8388 DAC volume 0-100 */
} settings_t;

#define SETTINGS_NAME_MAX 32

extern settings_t g_settings;
extern char g_dev_name[SETTINGS_NAME_MAX]; /* Bluetooth device name */

void settings_load(void);
void settings_save(void);
void settings_set_name(const char *name); /* NULL or "" = default */
