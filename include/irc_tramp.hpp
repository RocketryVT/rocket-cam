#pragma once

#include <cstdint>
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// IRC Tramp protocol — 9600 8N1, single-wire half-duplex via PIO
#define TRAMP_BAUD              9600
#define TRAMP_FRAME_LEN         16      // all frames are exactly 16 bytes

#define TRAMP_SYNC_START        0x0F
#define TRAMP_SYNC_STOP         0x00

// Commands — no response returned by VTX
#define TRAMP_CMD_SET_FREQ      'F'
#define TRAMP_CMD_SET_POWER     'P'
// 'I' param: 0 = RF off (pit mode on), 1 = RF on (pit mode off)
#define TRAMP_CMD_SET_ACTIVE    'I'
// Queries — VTX sends a 16-byte response
#define TRAMP_CMD_GET_CONFIG    'v'     // freq, power, pit mode, actual power
#define TRAMP_CMD_INIT_RF       'r'     // freq limits, max power

#define TRAMP_RESPONSE_TIMEOUT_MS   200

// Frequency limits returned by 'r' response
struct TrampRFLimits {
    uint16_t freq_min;   // MHz
    uint16_t freq_max;   // MHz
    uint16_t power_max;  // mW
};

// Current VTX status returned by 'v' response
struct TrampStatus {
    uint16_t frequency;      // MHz
    uint16_t conf_power;     // mW (configured)
    uint8_t  control_mode;   // bit 0 = race lock
    uint8_t  pit_mode;       // 1 = pit mode active
    uint16_t actual_power;   // mW (actual output)
};

// Single-wire half-duplex IRC Tramp driver using PIO.
//
// Uses two PIO state machines on the same GPIO pin — one for TX, one for RX.
// Only one is active at a time; direction is switched between send and receive.
//
// Typical usage:
//   IrcTramp vtx(pio0, 4);   // pio0, GPIO 4
//   vtx.begin();
//   vtx.init_rf(limits);
//   vtx.set_frequency(3330);
//   vtx.set_power(4000);
//   vtx.set_active(true);
class IrcTramp {
public:
    // pio:  pio0 or pio1
    // pin:  GPIO pin connected to the VTX signal wire
    IrcTramp(PIO pio, uint pin);

    // Load PIO programs and configure state machines. Call once on boot.
    void begin();

    // Send 'r' — read VTX frequency limits and max power into limits_out.
    // Returns false if no valid response within timeout.
    bool init_rf(TrampRFLimits& limits_out);

    // Send 'v' — read current freq, power, pit mode, actual power.
    // Returns false if no valid response or VTX echoed the request (freq == 0).
    bool get_config(TrampStatus& status_out);

    // Send 'F' — set transmit frequency in MHz. No response from VTX.
    bool set_frequency(uint16_t freq_mhz);

    // Send 'P' — set transmit power in mW. No response from VTX.
    bool set_power(uint16_t power_mw);

    // Send 'I' — enable or disable RF output.
    //   active=true  → RF on  (exit pit mode): sends param 1
    //   active=false → RF off (enter pit mode): sends param 0
    bool set_active(bool active);

private:
    PIO  _pio;
    uint _pin;
    uint _tx_sm;
    uint _rx_sm;
    uint _tx_offset;
    uint _rx_offset;

    void _enable_tx();
    void _enable_rx();
    void _flush_rx();

    // CRC: sum of bytes [1..13] (Betaflight vtx_tramp.c)
    static uint8_t _crc(const uint8_t* frame);

    void _build_frame(uint8_t cmd, uint16_t param, uint8_t* buf);
    bool _send(const uint8_t* buf);
    bool _send_recv(uint8_t cmd, uint16_t param, uint8_t* resp_out);
};
