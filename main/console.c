#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_gap_bt_api.h"
#include "esp_system.h"
#include "audio_io.h"
#include "bt_ag.h"
#include "console.h"
#include "ptt.h"
#include "settings.h"

#define CONSOLE_UART UART_NUM_0
#define CONSOLE_LINE_MAX 80

static void print_help(void)
{
    printf("\nBT-PTT commands:\n"
           "  status              show state and settings\n"
           "  scan                search headsets (pairing mode on the headset)\n"
           "  connect <n|addr>    connect to scan entry n or aa:bb:cc:dd:ee:ff\n"
           "  forget              forget the saved headset\n"
           "  ptt [on|off]        toggle / set PTT\n"
           "  mode call|idle      headset state (call: MFB sends CHUP, idle: BLDN/BVRA)\n"
           "  tot <s>             TX time-out in seconds (0 = off)\n"
           "  rxgain <dB>         radio -> headset gain (-20..20)\n"
           "  txgain <dB>         headset -> radio gain (-20..20)\n"
           "  outvol <0-100>      codec output level to the radio mic\n"
           "  name [text|-]       Bluetooth name (- = default)\n"
           "  meter               show audio peak levels for 5 s\n"
           "  reboot\n\n");
}

static void print_status(void)
{
    printf("\nboard   : %s\n", audio_io_board_name());
    printf("name    : %s\n", g_dev_name);
    bt_ag_print_status();
    printf("PTT     : %s\n", ptt_get() ? "ON (TX)" : "off (RX)");
    printf("mode    : %s\n", g_settings.mode == HS_MODE_CALL ? "call" : "idle");
    printf("tot     : %u s\n", g_settings.tot_s);
    printf("rxgain  : %d dB\n", g_settings.rx_gain_db);
    printf("txgain  : %d dB\n", g_settings.tx_gain_db);
    printf("outvol  : %u\n\n", g_settings.out_vol);
}

static int peak_dbfs(int16_t pk)
{
    if (pk <= 0) {
        return -99;
    }
    int db = 0;
    int32_t ref = 32767;
    while (ref > pk && db > -96) {
        ref = ref * 891 / 1000; /* -1 dB */
        db--;
    }
    return db;
}

static void meter(void)
{
    int16_t r, h;
    audio_io_get_peaks(&r, &h);
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        audio_io_get_peaks(&r, &h);
        printf("radio in %4d dBFS   headset mic %4d dBFS\n", peak_dbfs(r), peak_dbfs(h));
    }
}

static bool parse_addr(const char *s, uint8_t out[6])
{
    unsigned int v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xff) {
            return false;
        }
        out[i] = (uint8_t)v[i];
    }
    return true;
}

static bool parse_int(const char *s, int lo, int hi, int *out)
{
    if (!s || !*s) {
        return false;
    }
    char *end;
    long v = strtol(s, &end, 10);
    if (*end || v < lo || v > hi) {
        printf("value must be %d..%d\n", lo, hi);
        return false;
    }
    *out = (int)v;
    return true;
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    char *arg = strtok(NULL, " \t");
    int v;
    if (!cmd) {
        return;
    }
    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        print_help();
    } else if (!strcmp(cmd, "status")) {
        print_status();
    } else if (!strcmp(cmd, "scan")) {
        bt_ag_scan(false);
    } else if (!strcmp(cmd, "connect") && arg) {
        uint8_t addr[6];
        if (parse_addr(arg, addr)) {
            bt_ag_connect_addr(addr);
        } else if (parse_int(arg, 0, 99, &v)) {
            bt_ag_connect_index(v);
        }
    } else if (!strcmp(cmd, "forget")) {
        bt_ag_forget();
    } else if (!strcmp(cmd, "ptt")) {
        if (!arg) {
            ptt_toggle("console");
        } else {
            ptt_set(!strcmp(arg, "on"), "console");
        }
    } else if (!strcmp(cmd, "mode") && arg && (!strcmp(arg, "call") || !strcmp(arg, "idle"))) {
        g_settings.mode = !strcmp(arg, "call") ? HS_MODE_CALL : HS_MODE_IDLE;
        settings_save();
        bt_ag_mode_changed();
        printf("mode %s\n", arg);
    } else if (!strcmp(cmd, "tot") && parse_int(arg, 0, 600, &v)) {
        g_settings.tot_s = (uint16_t)v;
        settings_save();
        printf("tot %d s\n", v);
    } else if (!strcmp(cmd, "rxgain") && parse_int(arg, -20, 20, &v)) {
        g_settings.rx_gain_db = (int8_t)v;
        settings_save();
        audio_io_apply_settings();
    } else if (!strcmp(cmd, "txgain") && parse_int(arg, -20, 20, &v)) {
        g_settings.tx_gain_db = (int8_t)v;
        settings_save();
        audio_io_apply_settings();
    } else if (!strcmp(cmd, "outvol") && parse_int(arg, 0, 100, &v)) {
        g_settings.out_vol = (uint8_t)v;
        settings_save();
        audio_io_apply_settings();
    } else if (!strcmp(cmd, "name")) {
        if (arg) {
            settings_set_name(strcmp(arg, "-") ? arg : NULL);
            esp_bt_gap_set_device_name(g_dev_name);
        }
        printf("name %s\n", g_dev_name);
    } else if (!strcmp(cmd, "meter")) {
        meter();
    } else if (!strcmp(cmd, "reboot")) {
        esp_restart();
    } else {
        printf("? (type 'help')\n");
    }
}

static void console_task(void *arg)
{
    char line[CONSOLE_LINE_MAX];
    int len = 0;
    print_help();
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len == 0) {
                continue;
            }
            printf("\n");
            line[len] = 0;
            len = 0;
            handle_line(line);
        } else if (c == 0x08 || c == 0x7f) {
            if (len > 0) {
                len--;
                printf("\b \b");
            }
        } else if (c >= 0x20 && len < CONSOLE_LINE_MAX - 1) {
            line[len++] = (char)c;
            putchar(c);
        }
        fflush(stdout);
    }
}

void console_start(void)
{
    uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL, 0);
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
}
