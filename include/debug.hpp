#pragma once

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "pico/stdlib.h"
#include "tusb.h"
#include "irc_tramp.hpp"
#include "osd.hpp"
#include "pins.hpp"

// =============================================================================
// USB debug console  (compiled in only when DEBUG_USB is defined)
//
// Call debug_init() once after stdio_init_all(), then call debug_poll() from
// the main loop.  All I/O is non-blocking so it never stalls flight logic.
//
// Commands (newline-terminated, case-insensitive):
//   help                    — print this list
//   status                  — print state, altitude, camera, VTX
//   state <name>            — force state: PAD BOOST COAST APOGEE RECOVERY END
//   vtxfreq <MHz>           — set VTX frequency directly (e.g. 3330)
//   vtxband <A|B> <1-8>     — set VTX band/channel using table
//   vtxpower <mW>           — set VTX power (0 25 200 1000 4000)
//   vtxrf <on|off>          — enable / disable VTX RF output
//   vtxstatus               — query and print VTX config
//   osd <on|off>            — start / stop OSD heartbeat (calls osd.begin)
//   cam                     — manually trigger one camera toggle pulse
//   arm                     — simulate launch (transition PAD→BOOST)
//   land                    — simulate landing (transition to END)
// =============================================================================

// Forward declarations for globals defined in rocket-cam.cpp
extern volatile float     altitude;
extern volatile float     prev_altitude;
extern volatile float     ground_altitude;
extern volatile state_t   state;
extern volatile bool      enabled_camera;
extern IrcTramp           vtx;
extern Osd                osd;
int64_t toggle_camera_callback(alarm_id_t id, void* user_data);

static void _dbg_print_help() {
    printf("\r\n--- rocket-cam debug console ---\r\n");
    printf("  help\r\n");
    printf("  status\r\n");
    printf("  state <PAD|BOOST|COAST|APOGEE|RECOVERY|END>\r\n");
    printf("  vtxfreq <MHz>          e.g. vtxfreq 3330\r\n");
    printf("  vtxband <A|B> <1-8>    e.g. vtxband A 1\r\n");
    printf("  vtxpower <mW>          0 25 200 1000 4000\r\n");
    printf("  vtxrf <on|off>\r\n");
    printf("  vtxstatus\r\n");
    printf("  osd <on|off>\r\n");
    printf("  cam\r\n");
    printf("  arm\r\n");
    printf("  land\r\n");
    printf("--------------------------------\r\n");
}

static void _dbg_print_status() {
    const char* state_names[] = {"PAD","BOOST","COAST","APOGEE","RECOVERY","END"};
    printf("[STATUS] state=%s  alt=%.2fm  agl=%.2fm  climb=%.2fm/s  cam=%s\r\n",
           state_names[(int)state],
           (double)altitude,
           (double)(altitude - ground_altitude),
           (double)(altitude - prev_altitude),
           enabled_camera ? "ON" : "OFF");
}

static void _dbg_cmd_state(const char* arg) {
    if      (strcasecmp(arg, "PAD")      == 0) state = PAD;
    else if (strcasecmp(arg, "BOOST")    == 0) state = BOOST;
    else if (strcasecmp(arg, "COAST")    == 0) state = COAST;
    else if (strcasecmp(arg, "APOGEE")   == 0) state = APOGEE;
    else if (strcasecmp(arg, "RECOVERY") == 0) state = RECOVERY;
    else if (strcasecmp(arg, "END")      == 0) state = END;
    else { printf("[DEBUG] unknown state '%s'\r\n", arg); return; }
    printf("[DEBUG] state forced to %s\r\n", arg);
}

static void _dbg_cmd_vtxfreq(const char* arg) {
    int freq = atoi(arg);
    if (freq < 3000 || freq > 5900) {
        printf("[DEBUG] bad frequency %d MHz\r\n", freq);
        return;
    }
    vtx.set_frequency((uint16_t)freq);
    printf("[DEBUG] VTX frequency set to %d MHz\r\n", freq);
}

static void _dbg_cmd_vtxband(const char* band_str, const char* ch_str) {
    if (!band_str || !ch_str) { printf("[DEBUG] usage: vtxband <A|B> <1-8>\r\n"); return; }

    VtxBand band;
    if      (strcasecmp(band_str, "A") == 0) band = VtxBand::A;
    else if (strcasecmp(band_str, "B") == 0) band = VtxBand::B;
    else { printf("[DEBUG] unknown band '%s'\r\n", band_str); return; }

    int ch_num = atoi(ch_str);
    if (ch_num < 1 || ch_num > 8) { printf("[DEBUG] channel must be 1-8\r\n"); return; }
    VtxChannel ch = static_cast<VtxChannel>(ch_num - 1);

    uint16_t freq = vtx_frequency(band, ch);
    vtx.set_frequency(freq);
    printf("[DEBUG] VTX set to band %s ch%d (%d MHz)\r\n", band_str, ch_num, freq);
}

static void _dbg_cmd_vtxpower(const char* arg) {
    int mw = atoi(arg);
    uint16_t power;
    switch (mw) {
        case 0:    power = vtx_power_mw(VtxPower::PIT);    break;
        case 25:   power = vtx_power_mw(VtxPower::MW25);   break;
        case 200:  power = vtx_power_mw(VtxPower::MW200);  break;
        case 1000: power = vtx_power_mw(VtxPower::MW1000); break;
        case 4000: power = vtx_power_mw(VtxPower::MW4000); break;
        default:
            printf("[DEBUG] power must be one of: 0 25 200 1000 4000\r\n");
            return;
    }
    vtx.set_power(power);
    printf("[DEBUG] VTX power set to %d mW\r\n", mw);
}

static void _dbg_cmd_vtxrf(const char* arg) {
    if (strcasecmp(arg, "on") == 0) {
        vtx.set_active(true);
        printf("[DEBUG] VTX RF on\r\n");
    } else if (strcasecmp(arg, "off") == 0) {
        vtx.set_active(false);
        printf("[DEBUG] VTX RF off (pit mode)\r\n");
    } else {
        printf("[DEBUG] usage: vtxrf <on|off>\r\n");
    }
}

static void _dbg_cmd_vtxstatus() {
    TrampStatus s;
    if (!vtx.get_config(s)) {
        printf("[DEBUG] VTX did not respond to status query\r\n");
        return;
    }
    printf("[DEBUG] VTX freq=%u MHz  conf_power=%u mW  actual_power=%u mW  pit=%s  race_lock=%s\r\n",
           s.frequency, s.conf_power, s.actual_power,
           s.pit_mode     ? "yes" : "no",
           (s.control_mode & 1) ? "yes" : "no");
}

static bool _dbg_osd_enabled = true;

static void _dbg_cmd_osd(const char* arg) {
    if (strcasecmp(arg, "on") == 0) {
        if (!_dbg_osd_enabled) { osd.begin(); _dbg_osd_enabled = true; }
        printf("[DEBUG] OSD enabled\r\n");
    } else if (strcasecmp(arg, "off") == 0) {
        _dbg_osd_enabled = false;
        printf("[DEBUG] OSD disabled (heartbeats will stop on next update)\r\n");
    } else {
        printf("[DEBUG] usage: osd <on|off>\r\n");
    }
}

// Buffer for assembling an incoming line over multiple poll() calls
static char   _dbg_buf[64];
static uint8_t _dbg_len = 0;

static void _dbg_dispatch(char* line) {
    // Trim trailing CR/LF/spaces
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';
    if (len == 0) return;

    // Echo back
    printf("> %s\r\n", line);

    // Tokenise
    char* cmd  = strtok(line, " ");
    char* arg1 = strtok(NULL, " ");
    char* arg2 = strtok(NULL, " ");

    if      (strcasecmp(cmd, "help")      == 0) _dbg_print_help();
    else if (strcasecmp(cmd, "status")    == 0) _dbg_print_status();
    else if (strcasecmp(cmd, "state")     == 0) { if (arg1) _dbg_cmd_state(arg1);     else printf("[DEBUG] usage: state <name>\r\n"); }
    else if (strcasecmp(cmd, "vtxfreq")   == 0) { if (arg1) _dbg_cmd_vtxfreq(arg1);  else printf("[DEBUG] usage: vtxfreq <MHz>\r\n"); }
    else if (strcasecmp(cmd, "vtxband")   == 0) _dbg_cmd_vtxband(arg1, arg2);
    else if (strcasecmp(cmd, "vtxpower")  == 0) { if (arg1) _dbg_cmd_vtxpower(arg1); else printf("[DEBUG] usage: vtxpower <mW>\r\n"); }
    else if (strcasecmp(cmd, "vtxrf")     == 0) { if (arg1) _dbg_cmd_vtxrf(arg1);    else printf("[DEBUG] usage: vtxrf <on|off>\r\n"); }
    else if (strcasecmp(cmd, "vtxstatus") == 0) _dbg_cmd_vtxstatus();
    else if (strcasecmp(cmd, "osd")       == 0) { if (arg1) _dbg_cmd_osd(arg1);      else printf("[DEBUG] usage: osd <on|off>\r\n"); }
    else if (strcasecmp(cmd, "cam")       == 0) { toggle_camera_callback(0, NULL); printf("[DEBUG] camera toggle triggered\r\n"); }
    else if (strcasecmp(cmd, "arm")       == 0) { state = BOOST; printf("[DEBUG] launch simulated — state=BOOST\r\n"); }
    else if (strcasecmp(cmd, "land")      == 0) { state = END;   printf("[DEBUG] landing simulated — state=END\r\n"); }
    else printf("[DEBUG] unknown command '%s' — try 'help'\r\n", cmd);
}

// Call after cyw43_arch_init() so stdio_usb doesn't race for the same IRQ.
static inline void debug_init() {
    stdio_init_all();
    sleep_ms(1500);  // let USB enumerate
    printf("\r\n=== rocket-cam DEBUG BUILD ===\r\n");
    _dbg_print_help();
    printf("> ");
    stdio_flush();
}

// Call every main-loop iteration.  Never blocks.
static inline void debug_poll() {
    tud_task();  // pump TinyUSB — needed on Pico W with cyw43_arch_none
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (_dbg_len == 0) continue;  // skip empty lines
            _dbg_buf[_dbg_len] = '\0';
            _dbg_dispatch(_dbg_buf);
            _dbg_len = 0;
            printf("> ");
            stdio_flush();
        } else if (c == 127 || c == '\b') {  // backspace
            if (_dbg_len > 0) { _dbg_len--; printf("\b \b"); stdio_flush(); }
        } else if (_dbg_len < (uint8_t)(sizeof(_dbg_buf) - 1)) {
            _dbg_buf[_dbg_len++] = (char)c;
            putchar(c);  // local echo
            stdio_flush();
        }
    }
}
