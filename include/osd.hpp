#pragma once

#include <cstdint>
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

// MAVLink v1 OSD driver for Holybro Micro OSD V2.
//
// Sends two message types on a standard full-duplex hardware UART:
//   HEARTBEAT (msg 0)  — 1 Hz, keeps the OSD active
//   VFR_HUD   (msg 74) — altitude (m AGL) and climb rate (m/s)
//
// Usage:
//   Osd osd(uart0, PIN_OSD_TX, PIN_OSD_RX);
//   osd.begin();
//   // called at ~1 Hz from flight_task:
//   osd.update(altitude_m, climb_ms, custom_mode, line_a, line_b);
class Osd {
public:
    Osd(uart_inst_t* uart, uint pin_tx, uint pin_rx);

    // Initialise UART and GPIO. Call once on boot (before scheduler or from a task).
    void begin();

    // Send a HEARTBEAT if ≥1 s has elapsed since the last one, then VFR_HUD.
    // Alternates status_line_a / status_line_b in the OSD statustext overlay.
    void update(float altitude_m, float climb_ms, uint32_t custom_mode,
                const char* status_line_a, const char* status_line_b);

private:
    uart_inst_t* _uart;
    uint         _pin_tx;
    uint         _pin_rx;
    TickType_t   _last_heartbeat_tick;
    bool         _status_page;

    void _write_frame(const uint8_t* data, uint16_t length);
    void _send_heartbeat(uint32_t custom_mode);
    void _send_statustext(const char* text);
    void _send_vfr_hud(float alt, float climb);
};
