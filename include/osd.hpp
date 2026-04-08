#pragma once

#include <cstdint>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// MAVLink v1 OSD driver for Holybro Micro OSD V2.
//
// Sends two message types on a standard full-duplex hardware UART:
//   HEARTBEAT (msg 0)  — 1 Hz, keeps the OSD active
//   VFR_HUD   (msg 74) — whenever update() is called with new data,
//                        carries altitude (m) and climb rate (m/s)
//
// Usage:
//   Osd osd(OSD_UART, PIN_OSD_TX, PIN_OSD_RX);
//   osd.begin();
//   // in main loop or timer callback:
//   osd.update(altitude_m, climb_ms);
class Osd {
public:
    Osd(uart_inst_t* uart, uint pin_tx, uint pin_rx);

    // Initialise UART and GPIO. Call once on boot.
    void begin();

    // Send a HEARTBEAT if 1 s has elapsed, then send VFR_HUD with the
    // provided altitude (metres above ground) and vertical speed (m/s,
    // positive = climbing). Alternates the provided status text lines once
    // per second for the OSD message overlay.
    void update(float altitude_m, float climb_ms, uint32_t custom_mode,
                const char* status_line_a, const char* status_line_b);

private:
    uart_inst_t* _uart;
    uint         _pin_tx;
    uint         _pin_rx;
    absolute_time_t _last_heartbeat;
    bool         _status_page;

    void _write_frame(const uint8_t* data, uint16_t length);

    void _send_heartbeat(uint32_t custom_mode);
    void _send_statustext(const char* text);
    void _send_vfr_hud(float alt, float climb);
};
