#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t bt_ag_init(void);

/* Search for headsets (about 10 s). auto_connect: connect to the best headset found. */
void bt_ag_scan(bool auto_connect);
/* Connect to an entry of the last scan list (index) or to an address. */
void bt_ag_connect_index(int index);
void bt_ag_connect_addr(const uint8_t addr[6]);
/* Disconnect and forget the saved headset. */
void bt_ag_forget(void);
/* Re-send call indicators after g_settings.mode changed. */
void bt_ag_mode_changed(void);
void bt_ag_print_status(void);
bool bt_ag_audio_connected(void);
