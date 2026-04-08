#include "irc_tramp.hpp"
#include "uart_half_duplex.pio.h"
#include <cstring>

// ---------------------------------------------------------------------------
// PIO half-duplex single-wire UART
//
// Two state machines share one GPIO pin:
//   _tx_sm  - loaded with uart_hd_tx program, drives the pin as output
//   _rx_sm  - loaded with uart_hd_rx program, samples the pin as input
//
// Only one SM is enabled at a time.  _enable_tx() disables RX and re-asserts
// the pin as an output.  _enable_rx() disables TX and releases the pin to
// input so the VTX response can be sampled.
// ---------------------------------------------------------------------------

IrcTramp::IrcTramp(PIO pio, uint pin)
    : _pio(pio), _pin(pin), _tx_sm(0), _rx_sm(1),
      _tx_offset(0), _rx_offset(0) {}

void IrcTramp::begin() {
    // Claim two state machines
    _tx_sm = pio_claim_unused_sm(_pio, true);
    _rx_sm = pio_claim_unused_sm(_pio, true);

    // Load programs into PIO instruction memory
    _tx_offset = pio_add_program(_pio, &uart_hd_tx_program);
    _rx_offset = pio_add_program(_pio, &uart_hd_rx_program);

    // Initialise TX SM (starts enabled, pin as output, line high/idle)
    uart_hd_tx_program_init(_pio, _tx_sm, _tx_offset, _pin, TRAMP_BAUD);

    // Initialise RX SM but leave it disabled until we need to receive
    uart_hd_rx_program_init(_pio, _rx_sm, _rx_offset, _pin, TRAMP_BAUD);
    pio_sm_set_enabled(_pio, _rx_sm, false);
}

// ---- direction control ----------------------------------------------------

void IrcTramp::_enable_tx() {
    pio_sm_set_enabled(_pio, _rx_sm, false);
    // Restart TX SM so it's in the idle state (line high)
    pio_sm_restart(_pio, _tx_sm);
    pio_sm_exec(_pio, _tx_sm, pio_encode_jmp(_tx_offset));
    pio_sm_set_enabled(_pio, _tx_sm, true);
    // Pin back to output
    pio_sm_set_pindirs_with_mask(_pio, _tx_sm, 1u << _pin, 1u << _pin);
}

void IrcTramp::_enable_rx() {
    pio_sm_set_enabled(_pio, _tx_sm, false);
    // Pin to input before starting RX SM
    pio_sm_set_pindirs_with_mask(_pio, _rx_sm, 0u, 1u << _pin);
    gpio_pull_up(_pin);
    _flush_rx();
    pio_sm_restart(_pio, _rx_sm);
    pio_sm_exec(_pio, _rx_sm, pio_encode_jmp(_rx_offset));
    pio_sm_set_enabled(_pio, _rx_sm, true);
}

void IrcTramp::_flush_rx() {
    while (!pio_sm_is_rx_fifo_empty(_pio, _rx_sm)) {
        (void)pio_sm_get(_pio, _rx_sm);
    }
}

// ---- CRC ------------------------------------------------------------------
// Betaflight vtx_tramp.c: sum bytes [1..13] of the 16-byte frame.

uint8_t IrcTramp::_crc(const uint8_t* frame) {
    uint8_t cksum = 0;
    for (int i = 1; i <= 13; i++) {
        cksum += frame[i];
    }
    return cksum;
}

// ---- Frame construction ---------------------------------------------------

void IrcTramp::_build_frame(uint8_t cmd, uint16_t param, uint8_t* buf) {
    memset(buf, 0, TRAMP_FRAME_LEN);
    buf[0]  = TRAMP_SYNC_START;
    buf[1]  = cmd;
    buf[2]  = param & 0xFF;
    buf[3]  = (param >> 8) & 0xFF;
    buf[14] = _crc(buf);
    buf[15] = TRAMP_SYNC_STOP;
}

// ---- Send (no response expected) -----------------------------------------

bool IrcTramp::_send(const uint8_t* buf) {
    _enable_tx();
    for (int i = 0; i < TRAMP_FRAME_LEN; i++) {
        // Block until TX FIFO has space
        while (pio_sm_is_tx_fifo_full(_pio, _tx_sm)) { tight_loop_contents(); }
        pio_sm_put(_pio, _tx_sm, (uint32_t)buf[i]);
    }
    // Wait for TX SM to drain (FIFO empty + shift register done).
    // At 9600 baud, 16 bytes = 160 bits ≈ 16.7 ms — wait a safe 20 ms.
    sleep_ms(20);
    return true;
}

// ---- Send + receive -------------------------------------------------------
// Byte-by-byte state machine matching Betaflight's trampReceive() logic.

bool IrcTramp::_send_recv(uint8_t cmd, uint16_t param, uint8_t* resp_out) {
    uint8_t tx_buf[TRAMP_FRAME_LEN];
    _build_frame(cmd, param, tx_buf);
    _send(tx_buf);

    _enable_rx();

    enum { S_WAIT_SYNC, S_WAIT_CODE, S_DATA } state = S_WAIT_SYNC;
    uint8_t rx_buf[TRAMP_FRAME_LEN];
    int pos = 0;

    absolute_time_t deadline = make_timeout_time_ms(TRAMP_RESPONSE_TIMEOUT_MS);

    while (!time_reached(deadline)) {
        if (pio_sm_is_rx_fifo_empty(_pio, _rx_sm)) {
            continue;
        }
        // shift-right + autopush at 8: byte is in bits [31:24] of the FIFO word
        uint8_t c = (uint8_t)(pio_sm_get(_pio, _rx_sm) >> 24);

        switch (state) {
        case S_WAIT_SYNC:
            if (c == TRAMP_SYNC_START) {
                memset(rx_buf, 0, TRAMP_FRAME_LEN);
                rx_buf[0] = c;
                pos = 1;
                state = S_WAIT_CODE;
            }
            break;

        case S_WAIT_CODE:
            if (c == 'r' || c == 'v' || c == 's') {
                rx_buf[pos++] = c;
                state = S_DATA;
            } else {
                state = S_WAIT_SYNC;
            }
            break;

        case S_DATA:
            rx_buf[pos++] = c;
            if (pos == TRAMP_FRAME_LEN) {
                _enable_tx();
                if (rx_buf[14] != _crc(rx_buf) || rx_buf[15] != TRAMP_SYNC_STOP) {
                    return false;
                }
                if (resp_out) {
                    memcpy(resp_out, rx_buf, TRAMP_FRAME_LEN);
                }
                return true;
            }
            break;
        }
    }

    _enable_tx();
    return false;  // timeout
}

// ---- Public API -----------------------------------------------------------

bool IrcTramp::init_rf(TrampRFLimits& limits_out) {
    uint8_t resp[TRAMP_FRAME_LEN];
    if (!_send_recv(TRAMP_CMD_INIT_RF, 0, resp)) {
        return false;
    }
    // 'r' response: [2..3] freq_min, [4..5] freq_max, [6..7] power_max (all LE uint16)
    limits_out.freq_min  = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    if (limits_out.freq_min == 0) {
        return false;  // VTX echoed our request, not a real response
    }
    limits_out.freq_max  = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    limits_out.power_max = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
    return true;
}

bool IrcTramp::get_config(TrampStatus& status_out) {
    uint8_t resp[TRAMP_FRAME_LEN];
    if (!_send_recv(TRAMP_CMD_GET_CONFIG, 0, resp)) {
        return false;
    }
    // 'v' response: [2..3] freq, [4..5] conf_power, [6] ctrl_mode,
    //               [7] pit_mode, [8..9] actual_power (all LE)
    status_out.frequency    = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    if (status_out.frequency == 0) {
        return false;  // echo of our request
    }
    status_out.conf_power   = (uint16_t)resp[4] | ((uint16_t)resp[5] << 8);
    status_out.control_mode = resp[6];
    status_out.pit_mode     = resp[7];
    status_out.actual_power = (uint16_t)resp[8] | ((uint16_t)resp[9] << 8);
    return true;
}

bool IrcTramp::set_frequency(uint16_t freq_mhz) {
    uint8_t buf[TRAMP_FRAME_LEN];
    _build_frame(TRAMP_CMD_SET_FREQ, freq_mhz, buf);
    return _send(buf);
}

bool IrcTramp::set_power(uint16_t power_mw) {
    uint8_t buf[TRAMP_FRAME_LEN];
    _build_frame(TRAMP_CMD_SET_POWER, power_mw, buf);
    return _send(buf);
}

bool IrcTramp::set_active(bool active) {
    // param 1 = RF on (exit pit mode), param 0 = RF off (enter pit mode)
    uint8_t buf[TRAMP_FRAME_LEN];
    _build_frame(TRAMP_CMD_SET_ACTIVE, active ? 1u : 0u, buf);
    return _send(buf);
}
