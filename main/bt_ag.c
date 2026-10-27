#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "esp_hf_ag_legacy_api.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "audio_io.h"
#include "bt_ag.h"
#include "ptt.h"
#include "settings.h"
#include "tone.h"

#define TAG "bt_ag"

#define MAX_FOUND             10
#define SCAN_LEN_1280MS       8      /* inquiry length: 8 x 1.28 s */
#define BUTTON_DEBOUNCE_US    700000 /* headsets sometimes send 2 commands per press */
#define SCO_OPEN_DELAY_US     500000
#define SCO_RETRY_DELAY_US    1000000
#define SCO_LOST_PTT_OFF_US   2000000
#define FIRST_CONNECT_MS      4000   /* after boot: let the headset reconnect first */
#define RECONNECT_PERIOD_MS   8000
#define RECONNECT_JITTER_MS   3000
#define ACL_WAIT_FOR_HF_US    5000000
#define MAX_SCO_SAMPLES       128

typedef struct {
    esp_bd_addr_t addr;
    char          name[32];
    uint32_t      cod;
    int8_t        rssi;
} found_dev_t;

static found_dev_t s_found[MAX_FOUND];
static int s_found_n;
static volatile bool s_scanning;
static bool s_scan_auto_connect;
static bool s_connect_after_scan;
static esp_bd_addr_t s_connect_after_scan_addr;

static esp_bd_addr_t s_peer;
static volatile esp_hf_connection_state_t s_conn_state = ESP_HF_CONNECTION_STATE_DISCONNECTED;
static volatile esp_hf_audio_state_t s_audio_state = ESP_HF_AUDIO_STATE_DISCONNECTED;
static int64_t s_last_button_us;
static volatile bool s_acl_up;     /* baseband link to the saved headset exists */
static int64_t s_acl_up_since_us;
static volatile uint32_t s_rx_pkts, s_tx_pkts;

static esp_timer_handle_t s_sco_timer;
static esp_timer_handle_t s_sco_lost_timer;
static esp_timer_handle_t s_reconnect_timer;

static const char *const k_conn_str[] = { "disconnected", "connecting", "connected", "SLC connected", "disconnecting" };
static const char *const k_audio_str[] = { "disconnected", "connecting", "connected (CVSD)", "connected (mSBC)" };

static void addr_str(const uint8_t *a, char *out)
{
    sprintf(out, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], a[2], a[3], a[4], a[5]);
}

static bool slc_up(void)
{
    return s_conn_state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED;
}

bool bt_ag_audio_connected(void)
{
    return s_audio_state == ESP_HF_AUDIO_STATE_CONNECTED || s_audio_state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC;
}

static int call_indicator(void)
{
    return g_settings.mode == HS_MODE_CALL ? ESP_HF_CALL_STATUS_CALL_IN_PROGRESS : ESP_HF_CALL_STATUS_NO_CALLS;
}

/* ---------------------------------------------------------------- PTT button */

static void on_headset_button(const char *what)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_button_us < BUTTON_DEBOUNCE_US) {
        ESP_LOGI(TAG, "MFB: %s (ignored, repeat)", what);
        return;
    }
    s_last_button_us = now;
    ESP_LOGI(TAG, "MFB: %s", what);
    if (!slc_up()) {
        return;
    }
    /* Some headsets drop SCO themselves just before sending the command
     * (e.g. AT+BVRA): still toggle, and bring the audio back at once. */
    ptt_toggle(what);
    if (!bt_ag_audio_connected()) {
        esp_timer_stop(s_sco_timer);
        esp_timer_start_once(s_sco_timer, 0);
    }
}

/* SCO stayed down for SCO_LOST_PTT_OFF_US: stop transmitting. */
static void sco_lost_timer_cb(void *arg)
{
    if (!bt_ag_audio_connected()) {
        ptt_set(false, "SCO lost");
    }
}

/* ---------------------------------------------------------------- SCO audio */

/* Bytes the headset has sent us that we still owe it: every received byte is
 * answered with one sent byte, so both directions stay in step. */
static volatile uint32_t s_tx_credit;
static portMUX_TYPE s_credit_mux = portMUX_INITIALIZER_UNLOCKED;

/* SCO data from the headset (8 kHz 16-bit mono PCM, CVSD decoded by the controller). */
static void hf_incoming_cb(const uint8_t *buf, uint32_t len)
{
    int16_t pcm[MAX_SCO_SAMPLES];
    size_t n = len / 2;
    if (n > MAX_SCO_SAMPLES) {
        n = MAX_SCO_SAMPLES;
    }
    memcpy(pcm, buf, n * 2); /* buf is not necessarily 2-byte aligned */
    audio_io_from_headset(pcm, n);
    s_rx_pkts++;

    portENTER_CRITICAL(&s_credit_mux);
    s_tx_credit += len;
    if (s_tx_credit > 4 * MAX_SCO_SAMPLES * 2) {
        s_tx_credit = 4 * MAX_SCO_SAMPLES * 2;
    }
    portEXIT_CRITICAL(&s_credit_mux);
    esp_hf_ag_outgoing_data_ready();
}

/* The stack pulls data to send until we return less than len. */
static uint32_t hf_outgoing_cb(uint8_t *buf, uint32_t len)
{
    int16_t pcm[MAX_SCO_SAMPLES];
    size_t n = len / 2;
    if (n > MAX_SCO_SAMPLES) {
        return 0;
    }
    portENTER_CRITICAL(&s_credit_mux);
    bool ok = s_tx_credit >= len;
    if (ok) {
        s_tx_credit -= len;
    }
    portEXIT_CRITICAL(&s_credit_mux);
    if (!ok) {
        return 0;
    }
    audio_io_to_headset(pcm, n);
    memcpy(buf, pcm, n * 2);
    s_tx_pkts++;
    return len;
}

/* Debug: heap and SCO packet count every 30 s while audio is open. */
static void stats_timer_cb(void *arg)
{
    if (!bt_ag_audio_connected()) {
        return;
    }
    ESP_LOGI(TAG, "heap %u (min %u, largest %u)  sco rx %" PRIu32 " tx %" PRIu32 " /30s",
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), s_rx_pkts, s_tx_pkts);
    s_rx_pkts = 0;
    s_tx_pkts = 0;
}

/* Runs from esp_timer: (re)open the SCO link. */
static void sco_timer_cb(void *arg)
{
    if (!slc_up() || s_audio_state != ESP_HF_AUDIO_STATE_DISCONNECTED) {
        return;
    }
    if (g_settings.mode == HS_MODE_CALL) {
        esp_hf_ag_ciev_report(s_peer, ESP_HF_IND_TYPE_CALLSETUP, ESP_HF_CALL_SETUP_STATUS_IDLE);
        esp_hf_ag_ciev_report(s_peer, ESP_HF_IND_TYPE_CALL, ESP_HF_CALL_STATUS_CALL_IN_PROGRESS);
    }
    ESP_LOGI(TAG, "opening SCO audio");
    esp_hf_ag_audio_connect(s_peer);
}

static void schedule_reconnect(uint32_t base_ms)
{
    /* random jitter so that we rarely page at the same moment as the headset */
    uint64_t us = (base_ms + esp_random() % RECONNECT_JITTER_MS) * 1000ULL;
    esp_timer_stop(s_reconnect_timer);
    esp_timer_start_once(s_reconnect_timer, us);
}

/* Runs from esp_timer. The headset usually reconnects by itself after power-on;
 * only page it when no link exists, otherwise build HFP on the existing link. */
static void reconnect_timer_cb(void *arg)
{
    schedule_reconnect(RECONNECT_PERIOD_MS);
    if (!g_settings.has_peer || s_scanning || s_conn_state != ESP_HF_CONNECTION_STATE_DISCONNECTED) {
        return;
    }
    if (s_acl_up && esp_timer_get_time() - s_acl_up_since_us < ACL_WAIT_FOR_HF_US) {
        return; /* headset is connecting to us: give it time */
    }
    char a[18];
    addr_str(g_settings.peer, a);
    ESP_LOGI(TAG, "connecting to %s%s", a, s_acl_up ? " (existing link)" : "");
    memcpy(s_peer, g_settings.peer, ESP_BD_ADDR_LEN);
    esp_hf_ag_slc_connect(s_peer);
}

/* ---------------------------------------------------------------- HFP AG events */

static void hf_cb(esp_hf_cb_event_t event, esp_hf_cb_param_t *param)
{
    char a[18];
    switch (event) {
    case ESP_HF_CONNECTION_STATE_EVT:
        s_conn_state = param->conn_stat.state;
        addr_str(param->conn_stat.remote_bda, a);
        ESP_LOGI(TAG, "HFP %s: %s (peer feat 0x%" PRIx32 ")", a,
                 k_conn_str[param->conn_stat.state], param->conn_stat.peer_feat);
        if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED) {
            memcpy(s_peer, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            if (!g_settings.has_peer || memcmp(g_settings.peer, s_peer, ESP_BD_ADDR_LEN) != 0) {
                memcpy(g_settings.peer, s_peer, ESP_BD_ADDR_LEN);
                g_settings.has_peer = true;
                settings_save();
                ESP_LOGI(TAG, "headset %s saved", a);
            }
            esp_timer_stop(s_sco_timer);
            esp_timer_start_once(s_sco_timer, SCO_OPEN_DELAY_US);
        } else if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_DISCONNECTED) {
            s_audio_state = ESP_HF_AUDIO_STATE_DISCONNECTED;
            ptt_set(false, "headset disconnected");
        }
        break;

    case ESP_HF_AUDIO_STATE_EVT:
        s_audio_state = param->audio_stat.state;
        ESP_LOGI(TAG, "SCO audio %s (frame %u bytes)", k_audio_str[param->audio_stat.state],
                 param->audio_stat.preferred_frame_size);
        if (bt_ag_audio_connected()) {
            audio_io_reset_headset_path();
            s_tx_credit = 0;
            esp_hf_ag_register_data_callback(hf_incoming_cb, hf_outgoing_cb);
            tone_play(TONE_CONNECTED);
        } else if (param->audio_stat.state == ESP_HF_AUDIO_STATE_DISCONNECTED) {
            esp_timer_stop(s_sco_lost_timer);
            esp_timer_start_once(s_sco_lost_timer, SCO_LOST_PTT_OFF_US);
            if (slc_up()) {
                esp_timer_stop(s_sco_timer);
                esp_timer_start_once(s_sco_timer, SCO_RETRY_DELAY_US);
            }
        }
        break;

    /* ---- headset button (MFB): every one of these toggles PTT ---- */
    case ESP_HF_CHUP_RESPONSE_EVT: /* stack already replied OK */
        on_headset_button("AT+CHUP");
        break;
    case ESP_HF_ATA_RESPONSE_EVT:  /* stack already replied OK */
        on_headset_button("ATA");
        break;
    case ESP_HF_BVRA_RESPONSE_EVT: /* stack already replied OK */
        on_headset_button(param->vra_rep.value == ESP_HF_VR_STATE_ENABLED ? "AT+BVRA=1" : "AT+BVRA=0");
        break;
    case ESP_HF_DIAL_EVT:
        if (param->out_call.num_or_loc == NULL) {
            esp_hf_ag_cmee_send(param->out_call.remote_addr, ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            on_headset_button("AT+BLDN");
        } else {
            ESP_LOGI(TAG, "dial \"%s\" refused", param->out_call.num_or_loc);
            esp_hf_ag_cmee_send(param->out_call.remote_addr, ESP_HF_AT_RESPONSE_CODE_ERR, ESP_HF_CME_OPERATION_NOT_SUPPORTED);
        }
        break;
    case ESP_HF_UNAT_RESPONSE_EVT:
        ESP_LOGI(TAG, "unknown AT: %s", param->unat_rep.unat ? param->unat_rep.unat : "(null)");
        if (param->unat_rep.unat && strstr(param->unat_rep.unat, "CKPD")) {
            esp_hf_ag_cmee_send(param->unat_rep.remote_addr, ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            on_headset_button("AT+CKPD");
        } else {
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr, NULL);
        }
        break;

    /* ---- indicators / queries ---- */
    case ESP_HF_CIND_RESPONSE_EVT:
        esp_hf_ag_cind_response(param->cind_rep.remote_addr, call_indicator(), ESP_HF_CALL_SETUP_STATUS_IDLE,
                                ESP_HF_NETWORK_STATE_AVAILABLE, 5, ESP_HF_ROAMING_STATUS_INACTIVE, 5,
                                ESP_HF_CALL_HELD_STATUS_NONE);
        break;
    case ESP_HF_IND_UPDATE_EVT:
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALL, call_indicator());
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALLSETUP, ESP_HF_CALL_SETUP_STATUS_IDLE);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SERVICE, ESP_HF_NETWORK_STATE_AVAILABLE);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SIGNAL, 5);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_BATTCHG, 5);
        break;
    case ESP_HF_COPS_RESPONSE_EVT:
        esp_hf_ag_cops_response(param->cops_rep.remote_addr, "BT-PTT");
        break;
    case ESP_HF_CLCC_RESPONSE_EVT:
        if (g_settings.mode == HS_MODE_CALL) {
            esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, 1, ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
                                    ESP_HF_CURRENT_CALL_STATUS_ACTIVE, ESP_HF_CURRENT_CALL_MODE_VOICE,
                                    ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE, "0", ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        }
        /* index 0 terminates the list with OK */
        esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, 0, 0, 0, 0, 0, NULL, ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        break;
    case ESP_HF_CNUM_RESPONSE_EVT:
        esp_hf_ag_cmee_send(param->cnum_rep.remote_addr, ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
        break;
    case ESP_HF_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "headset %s volume %d",
                 param->volume_control.type == ESP_HF_VOLUME_TYPE_SPK ? "speaker" : "mic",
                 param->volume_control.volume);
        break;
    case ESP_HF_NREC_RESPONSE_EVT:
    case ESP_HF_VTS_RESPONSE_EVT:
    case ESP_HF_BCS_RESPONSE_EVT:
    case ESP_HF_WBS_RESPONSE_EVT:
    case ESP_HF_PKT_STAT_NUMS_GET_EVT:
        break;
    case ESP_HF_PROF_STATE_EVT:
        ESP_LOGI(TAG, "HFP AG profile state %d", param->prof_stat.state);
        break;
    default:
        ESP_LOGI(TAG, "HFP event %d", event);
        break;
    }
}

/* ---------------------------------------------------------------- GAP / scan */

static bool is_headset_cod(uint32_t cod)
{
    if (esp_bt_gap_get_cod_major_dev(cod) != ESP_BT_COD_MAJOR_DEV_AV) {
        return false;
    }
    uint32_t minor = esp_bt_gap_get_cod_minor_dev(cod);
    return minor == 1 /* wearable headset */ || minor == 2 /* hands-free */ || minor == 6 /* headphones */;
}

static void add_found(esp_bt_gap_cb_param_t *param)
{
    found_dev_t d = { 0 };
    memcpy(d.addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
    d.rssi = -127;
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            d.cod = *(uint32_t *)p->val;
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            d.rssi = *(int8_t *)p->val;
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            int len = p->len < (int)sizeof(d.name) - 1 ? p->len : (int)sizeof(d.name) - 1;
            memcpy(d.name, p->val, len);
            d.name[len] = 0;
            break;
        }
        case ESP_BT_GAP_DEV_PROP_EIR:
            if (!d.name[0]) {
                uint8_t len = 0;
                uint8_t *n = esp_bt_gap_resolve_eir_data(p->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
                if (!n) {
                    n = esp_bt_gap_resolve_eir_data(p->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
                }
                if (n) {
                    if (len > sizeof(d.name) - 1) {
                        len = sizeof(d.name) - 1;
                    }
                    memcpy(d.name, n, len);
                    d.name[len] = 0;
                }
            }
            break;
        default:
            break;
        }
    }
    for (int i = 0; i < s_found_n; i++) {
        if (memcmp(s_found[i].addr, d.addr, ESP_BD_ADDR_LEN) == 0) {
            if (!d.name[0]) {
                strcpy(d.name, s_found[i].name);
            }
            s_found[i] = d;
            return;
        }
    }
    if (s_found_n < MAX_FOUND) {
        s_found[s_found_n++] = d;
    }
}

static void print_found(void)
{
    printf("\n--- %d device(s) found ---\n", s_found_n);
    for (int i = 0; i < s_found_n; i++) {
        char a[18];
        addr_str(s_found[i].addr, a);
        printf(" [%d] %s  %4d dBm  cod 0x%06" PRIx32 " %s  %s\n", i, a, s_found[i].rssi, s_found[i].cod,
               is_headset_cod(s_found[i].cod) ? "HEADSET" : "       ", s_found[i].name);
    }
    printf("use 'connect <n>' to pair\n\n");
}

static void start_slc(const uint8_t *addr)
{
    char a[18];
    addr_str(addr, a);
    ESP_LOGI(TAG, "connecting to %s", a);
    memcpy(s_peer, addr, ESP_BD_ADDR_LEN);
    esp_hf_ag_slc_connect(s_peer);
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char a[18];
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        add_found(param);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "searching for headsets... (put the headset in pairing mode)");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            s_scanning = false;
            print_found();
            if (s_connect_after_scan) {
                s_connect_after_scan = false;
                start_slc(s_connect_after_scan_addr);
            } else if (s_scan_auto_connect) {
                int best = -1;
                for (int i = 0; i < s_found_n; i++) {
                    if (is_headset_cod(s_found[i].cod) && (best < 0 || s_found[i].rssi > s_found[best].rssi)) {
                        best = i;
                    }
                }
                if (best >= 0) {
                    start_slc(s_found[best].addr);
                } else {
                    ESP_LOGW(TAG, "no headset found");
                }
            }
        }
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        addr_str(param->auth_cmpl.bda, a);
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "paired with %s %s", a, (char *)param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "pairing with %s failed (%d)", a, param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        /* legacy pairing: headsets use 0000 */
        esp_bt_pin_code_t pin = { '0', '0', '0', '0' };
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        addr_str(param->acl_conn_cmpl_stat.bda, a);
        ESP_LOGI(TAG, "link to %s %s (0x%x)", a,
                 param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS ? "up" : "failed",
                 param->acl_conn_cmpl_stat.stat);
        if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS && g_settings.has_peer &&
            memcmp(param->acl_conn_cmpl_stat.bda, g_settings.peer, ESP_BD_ADDR_LEN) == 0) {
            s_acl_up = true;
            s_acl_up_since_us = esp_timer_get_time();
        }
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        addr_str(param->acl_disconn_cmpl_stat.bda, a);
        ESP_LOGI(TAG, "link to %s down (reason 0x%x)", a, param->acl_disconn_cmpl_stat.reason);
        if (g_settings.has_peer && memcmp(param->acl_disconn_cmpl_stat.bda, g_settings.peer, ESP_BD_ADDR_LEN) == 0) {
            s_acl_up = false;
        }
        break;
    default:
        break;
    }
}

/* ---------------------------------------------------------------- public */

esp_err_t bt_ag_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bd_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_gap_set_device_name(g_dev_name);
    esp_bt_gap_register_callback(gap_cb);
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap));
    esp_bt_pin_code_t pin = { 0 };
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin);

    ESP_ERROR_CHECK(esp_hf_ag_register_callback(hf_cb));
    ESP_ERROR_CHECK(esp_hf_ag_init());

    /* the headset may also reconnect by itself */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    const esp_timer_create_args_t sco_args = { .callback = sco_timer_cb, .name = "sco" };
    ESP_ERROR_CHECK(esp_timer_create(&sco_args, &s_sco_timer));
    const esp_timer_create_args_t rc_args = { .callback = reconnect_timer_cb, .name = "reconnect" };
    ESP_ERROR_CHECK(esp_timer_create(&rc_args, &s_reconnect_timer));
    const esp_timer_create_args_t lost_args = { .callback = sco_lost_timer_cb, .name = "sco_lost" };
    ESP_ERROR_CHECK(esp_timer_create(&lost_args, &s_sco_lost_timer));
    esp_timer_handle_t stats_timer;
    const esp_timer_create_args_t st_args = { .callback = stats_timer_cb, .name = "stats" };
    ESP_ERROR_CHECK(esp_timer_create(&st_args, &stats_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(stats_timer, 30000000));

    const uint8_t *own = esp_bt_dev_get_address();
    char a[18];
    addr_str(own, a);
    ESP_LOGI(TAG, "HFP AG ready: %s (%s)", g_dev_name, a);

    schedule_reconnect(FIRST_CONNECT_MS);
    if (!g_settings.has_peer) {
        ESP_LOGI(TAG, "no headset saved: searching automatically");
        bt_ag_scan(true);
    }
    return ESP_OK;
}

void bt_ag_scan(bool auto_connect)
{
    if (s_scanning) {
        return;
    }
    s_found_n = 0;
    s_scan_auto_connect = auto_connect;
    s_scanning = true;
    if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, SCAN_LEN_1280MS, 0) != ESP_OK) {
        s_scanning = false;
        ESP_LOGE(TAG, "cannot start discovery");
    }
}

void bt_ag_connect_addr(const uint8_t addr[6])
{
    if (s_conn_state != ESP_HF_CONNECTION_STATE_DISCONNECTED) {
        esp_hf_ag_slc_disconnect(s_peer);
    }
    if (s_scanning) {
        memcpy(s_connect_after_scan_addr, addr, ESP_BD_ADDR_LEN);
        s_connect_after_scan = true;
        esp_bt_gap_cancel_discovery();
        return;
    }
    start_slc(addr);
}

void bt_ag_connect_index(int index)
{
    if (index < 0 || index >= s_found_n) {
        printf("no such entry (run 'scan' first)\n");
        return;
    }
    bt_ag_connect_addr(s_found[index].addr);
}

void bt_ag_forget(void)
{
    if (g_settings.has_peer) {
        esp_bt_gap_remove_bond_device(g_settings.peer);
    }
    g_settings.has_peer = false;
    memset(g_settings.peer, 0, sizeof(g_settings.peer));
    settings_save();
    if (s_conn_state != ESP_HF_CONNECTION_STATE_DISCONNECTED) {
        esp_hf_ag_slc_disconnect(s_peer);
    }
    printf("headset forgotten\n");
}

void bt_ag_mode_changed(void)
{
    if (slc_up()) {
        esp_hf_ag_ciev_report(s_peer, ESP_HF_IND_TYPE_CALL, call_indicator());
    }
}

void bt_ag_print_status(void)
{
    char a[18] = "(none)";
    if (g_settings.has_peer) {
        addr_str(g_settings.peer, a);
    }
    printf("headset : %s\n", a);
    printf("HFP     : %s\n", k_conn_str[s_conn_state]);
    printf("SCO     : %s\n", k_audio_str[s_audio_state]);
    printf("scanning: %s\n", s_scanning ? "yes" : "no");
}
