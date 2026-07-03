#include "ble.hpp"

#include <cstring>

#include "pico/cyw43_arch.h"
#include "pico/async_context.h"

extern "C" {
#include "btstack.h"
#include "sigma2_cam.h"   // generated from sigma2_cam.gatt (profile_data + handles)
}

#include "pb_encode.h"
#include "pb_decode.h"
#include "SIGMA2_Proto.pb.h"

// ---------------------------------------------------------------------------
// GATT attribute handles (from the generated sigma2_cam.h)
// ---------------------------------------------------------------------------
#define CMD_VALUE_HANDLE \
    ATT_CHARACTERISTIC_9A8B0002_7C6D_4B2A_9E3F_1C2D3E4F5060_01_VALUE_HANDLE
#define STATUS_VALUE_HANDLE \
    ATT_CHARACTERISTIC_9A8B0003_7C6D_4B2A_9E3F_1C2D3E4F5060_01_VALUE_HANDLE
#define STATUS_CCC_HANDLE \
    ATT_CHARACTERISTIC_9A8B0003_7C6D_4B2A_9E3F_1C2D3E4F5060_01_CLIENT_CONFIGURATION_HANDLE

// ---------------------------------------------------------------------------
// Shared state between the flight loop and the BLE/async context.
//
// All access from the flight loop goes through the async-context lock. BTstack
// callbacks already run under that lock (threadsafe_background disables the
// background worker while a lock is held), so they touch these directly.
// ---------------------------------------------------------------------------
static CamCmd  s_cmd          = {};
static bool    s_cmd_pending  = false;

static CamCfg  s_cfg          = {};
static uint32_t s_cfg_version = 0;   // bumped on every publish
static uint32_t s_cfg_notified = 0;  // last version pushed as a notification

// ---------------------------------------------------------------------------
// BLE connection / advertising state
// ---------------------------------------------------------------------------
static hci_con_handle_t s_con_handle = HCI_CON_HANDLE_INVALID;
static bool             s_notify_on  = false;
static volatile bool    s_ble_ready  = false;
static volatile int     s_ble_init_result = PICO_ERROR_GENERIC;
static btstack_packet_callback_registration_t s_hci_cb;
static btstack_timer_source_t                 s_status_timer;

// Advertise flags + the camera-control service UUID. iOS service-filtered
// scans match the primary advertising payload more reliably than scan response
// data, so keep the UUID here and put the human-readable name in scan response.
static const uint8_t s_adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x60, 0x50, 0x4F, 0x3E, 0x2D, 0x1C, 0x3F, 0x9E,
    0x2A, 0x4B, 0x6D, 0x7C, 0x01, 0x00, 0x8B, 0x9A,
};

// Scan response carries the complete local name "RocketCam".
static const uint8_t s_scan_resp_data[] = {
    0x0A, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'R', 'o', 'c', 'k', 'e', 't', 'C', 'a', 'm',
};

// ---------------------------------------------------------------------------
// Encode the current config snapshot into an Envelope{cam_config}.
// Returns encoded length, 0 on failure.
// ---------------------------------------------------------------------------
static size_t encode_cfg(uint8_t* out, size_t out_size) {
    sigma2_Envelope env = sigma2_Envelope_init_zero;
    env.node_src      = sigma2_NodeId_NODE_ROCKET_CAM;
    env.node_dst      = sigma2_NodeId_NODE_UNDEFINED;
    env.which_payload = sigma2_Envelope_cam_config_tag;

    sigma2_CameraConfig& c = env.payload.cam_config;
    c.band             = (sigma2_VtxBand)s_cfg.band;
    c.channel          = s_cfg.channel;
    c.freq_mhz         = s_cfg.freq_mhz;
    c.power            = (sigma2_VtxPowerLevel)s_cfg.power;
    c.actual_power_mw  = s_cfg.actual_power_mw;
    c.rf_enabled       = s_cfg.rf_enabled;
    c.camera_recording = s_cfg.camera_recording;
    c.flight_state     = (sigma2_FlightState)s_cfg.flight_state;
    c.altitude_agl_m   = s_cfg.altitude_agl_m;
    c.vtx_responsive   = s_cfg.vtx_responsive;

    pb_ostream_t os = pb_ostream_from_buffer(out, out_size);
    if (!pb_encode(&os, sigma2_Envelope_fields, &env)) return 0;
    return os.bytes_written;
}

// ---------------------------------------------------------------------------
// Decode an incoming Envelope{cam_command} into the mailbox.
// ---------------------------------------------------------------------------
static void decode_cmd(const uint8_t* data, uint16_t len) {
    sigma2_Envelope env = sigma2_Envelope_init_zero;
    pb_istream_t is = pb_istream_from_buffer(data, len);
    if (!pb_decode(&is, sigma2_Envelope_fields, &env)) return;
    if (env.which_payload != sigma2_Envelope_cam_command_tag) return;

    const sigma2_CameraCommand& in = env.payload.cam_command;
    CamCmd c = {};
    if ((c.has_band = in.has_band))       c.band         = (uint8_t)in.band;
    if ((c.has_channel = in.has_channel)) c.channel      = (uint8_t)in.channel;
    if ((c.has_freq = in.has_freq_mhz))   c.freq_mhz     = (uint16_t)in.freq_mhz;
    if ((c.has_power = in.has_power))     c.power        = (uint8_t)in.power;
    if ((c.has_rf = in.has_rf_enabled))   c.rf_enabled   = in.rf_enabled;
    if ((c.has_record = in.has_camera_record)) c.camera_record = in.camera_record;
    c.request_status = in.has_request_status && in.request_status;

    // We are in the BLE/async context (lock held); write the mailbox directly.
    s_cmd         = c;
    s_cmd_pending = true;
}

// ---------------------------------------------------------------------------
// ATT read — STATUS characteristic returns the encoded config.
// ---------------------------------------------------------------------------
static uint16_t att_read_cb(hci_con_handle_t, uint16_t att_handle,
                            uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    if (att_handle == STATUS_VALUE_HANDLE) {
        uint8_t tmp[sigma2_Envelope_size];
        size_t n = encode_cfg(tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, (uint16_t)n, offset, buffer, buffer_size);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// ATT write — CMD characteristic decode, STATUS CCC notify enable.
// ---------------------------------------------------------------------------
static int att_write_cb(hci_con_handle_t con_handle, uint16_t att_handle,
                        uint16_t transaction_mode, uint16_t /*offset*/,
                        uint8_t* buffer, uint16_t buffer_size) {
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) return 0;

    if (att_handle == CMD_VALUE_HANDLE) {
        decode_cmd(buffer, buffer_size);
        return 0;
    }
    if (att_handle == STATUS_CCC_HANDLE) {
        s_notify_on  = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        s_con_handle = con_handle;
        return 0;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// HCI / ATT event handling
// ---------------------------------------------------------------------------
static void packet_handler(uint8_t packet_type, uint16_t, uint8_t* packet, uint16_t) {
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            s_con_handle = HCI_CON_HANDLE_INVALID;
            s_notify_on  = false;
            break;
        case ATT_EVENT_CAN_SEND_NOW: {
            if (s_con_handle == HCI_CON_HANDLE_INVALID || !s_notify_on) break;
            uint8_t tmp[sigma2_Envelope_size];
            size_t n = encode_cfg(tmp, sizeof(tmp));
            if (n) att_server_notify(s_con_handle, STATUS_VALUE_HANDLE, tmp, (uint16_t)n);
            s_cfg_notified = s_cfg_version;
            break;
        }
        default:
            break;
    }
}

// Periodic check: if the config changed and notifications are on, ask BTstack
// for a send slot. Runs in the BLE context (lock held).
static void status_timer_cb(btstack_timer_source_t* ts) {
    if (s_notify_on && s_con_handle != HCI_CON_HANDLE_INVALID &&
        s_cfg_version != s_cfg_notified) {
        att_server_request_can_send_now_event(s_con_handle);
    }
    btstack_run_loop_set_timer(ts, 500);
    btstack_run_loop_add_timer(ts);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool ble_init() {
    if (s_ble_ready) return true;

    s_ble_init_result = cyw43_arch_init();
    if (s_ble_init_result != 0) {
        return false;
    }

    l2cap_init();
    sm_init();

    att_server_init(profile_data, att_read_cb, att_write_cb);

    s_hci_cb.callback = &packet_handler;
    hci_add_event_handler(&s_hci_cb);
    att_server_register_packet_handler(&packet_handler);

    uint16_t adv_int_min = 0x0030;  // 30 ms
    uint16_t adv_int_max = 0x0060;  // 60 ms
    bd_addr_t null_addr; memset(null_addr, 0, sizeof(null_addr));
    gap_advertisements_set_params(adv_int_min, adv_int_max, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(sizeof(s_adv_data), (uint8_t*)s_adv_data);
    gap_scan_response_set_data(sizeof(s_scan_resp_data), (uint8_t*)s_scan_resp_data);
    gap_advertisements_enable(1);

    s_status_timer.process = &status_timer_cb;
    btstack_run_loop_set_timer(&s_status_timer, 500);
    btstack_run_loop_add_timer(&s_status_timer);

    hci_power_control(HCI_POWER_ON);
    s_ble_ready = true;
    return true;
}

bool ble_is_ready() {
    return s_ble_ready;
}

int ble_init_result() {
    return s_ble_init_result;
}

bool ble_take_cam_cmd(CamCmd* out) {
    if (!s_ble_ready || !out) return false;

    bool got = false;
    async_context_t* ctx = cyw43_arch_async_context();
    if (!ctx) return false;
    async_context_acquire_lock_blocking(ctx);
    if (s_cmd_pending) {
        *out          = s_cmd;
        s_cmd_pending = false;
        got           = true;
    }
    async_context_release_lock(ctx);
    return got;
}

void ble_publish_cam_cfg(const CamCfg* cfg) {
    if (!cfg) return;

    async_context_t* ctx = cyw43_arch_async_context();
    if (!s_ble_ready || !ctx) {
        s_cfg = *cfg;
        s_cfg_version++;
        return;
    }

    async_context_acquire_lock_blocking(ctx);
    s_cfg = *cfg;
    s_cfg_version++;
    async_context_release_lock(ctx);
}
