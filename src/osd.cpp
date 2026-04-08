#include "osd.hpp"
#include "c_library_v1/all/mavlink.h"

#include <cstring>

namespace {

constexpr uint8_t kMavSystemId = 1;
constexpr uint8_t kMavComponentId = MAV_COMP_ID_AUTOPILOT1;

}  // namespace

Osd::Osd(uart_inst_t* uart, uint pin_tx, uint pin_rx)
    : _uart(uart), _pin_tx(pin_tx), _pin_rx(pin_rx), _status_page(false) {}

void Osd::begin() {
    uart_init(_uart, 57600);
    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
    gpio_set_function(_pin_tx, GPIO_FUNC_UART);
    gpio_set_function(_pin_rx, GPIO_FUNC_UART);
    _last_heartbeat = nil_time;
    _status_page = false;
}

void Osd::_write_frame(const uint8_t* data, uint16_t length) {
    for (uint16_t index = 0; index < length; ++index) {
        uart_putc_raw(_uart, data[index]);
    }
}

void Osd::_send_heartbeat(uint32_t custom_mode) {
    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(
        kMavSystemId,
        kMavComponentId,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        0,
        custom_mode,
        MAV_STATE_ACTIVE);

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t frame_length = mavlink_msg_to_send_buffer(frame, &message);
    _write_frame(frame, frame_length);
}

void Osd::_send_statustext(const char* text) {
    if (text == nullptr || text[0] == '\0') {
        return;
    }

    mavlink_message_t message;
    mavlink_msg_statustext_pack(
        kMavSystemId,
        kMavComponentId,
        &message,
        MAV_SEVERITY_INFO,
        text);

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t frame_length = mavlink_msg_to_send_buffer(frame, &message);
    _write_frame(frame, frame_length);
}

void Osd::_send_vfr_hud(float alt, float climb) {
    mavlink_message_t message;
    mavlink_msg_vfr_hud_pack(
        kMavSystemId,
        kMavComponentId,
        &message,
        0.0f,
        0.0f,
        0,
        0,
        alt,
        climb);

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t frame_length = mavlink_msg_to_send_buffer(frame, &message);
    _write_frame(frame, frame_length);
}

void Osd::update(float altitude_m, float climb_ms, uint32_t custom_mode,
                 const char* status_line_a, const char* status_line_b) {
    // Send HEARTBEAT at 1 Hz
    if (is_nil_time(_last_heartbeat) ||
        absolute_time_diff_us(_last_heartbeat, get_absolute_time()) >= 1000000) {
        _send_heartbeat(custom_mode);
        _send_statustext(_status_page ? status_line_b : status_line_a);
        _status_page = !_status_page;
        _last_heartbeat = get_absolute_time();
    }

    _send_vfr_hud(altitude_m, climb_ms);
}
