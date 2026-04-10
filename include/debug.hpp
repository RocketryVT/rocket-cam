#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "pico/stdio_usb.h"

#include "irc_tramp.hpp"
#include "osd.hpp"
#include "shared.hpp"   // FlightState, NOTIF_*, queue externs

// ---------------------------------------------------------------------------
// Externs defined in rocket-cam.cpp
// ---------------------------------------------------------------------------

extern IrcTramp g_vtx;
extern Osd      g_osd;

// Requests a camera pulse from flight_task (GPIO stays on that task)
void camera_pulse_from_task();

// ---------------------------------------------------------------------------
// Command dispatcher — called with a null-terminated, trimmed line
// ---------------------------------------------------------------------------

static void _dbg_dispatch(char* line) {
    // ltrim
    while (*line == ' ') line++;
    int len = (int)strlen(line);
    // rtrim
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';
    if (len == 0) return;

    char* cmd  = strtok(line, " ");
    char* arg1 = strtok(nullptr, " ");
    char* arg2 = strtok(nullptr, " ");

    if (strcasecmp(cmd, "help") == 0) {
        printf("Commands:\r\n"
               "  status              print current flight state\r\n"
               "  state <S>           force state (PAD/BOOST/COAST/APOGEE/RECOVERY/END)\r\n"
               "  vtxfreq <MHz>       set VTX frequency\r\n"
               "  vtxband <A|B> <ch>  set VTX band+channel (ch 1-8)\r\n"
               "  vtxpower <mW>       set VTX power (0/25/200/1000/4000)\r\n"
               "  vtxrf <on|off>      enable/disable RF output\r\n"
               "  vtxstatus           query VTX current config\r\n"
               "  cam                 toggle camera record\r\n"
               "  arm                 alias: state BOOST\r\n"
               "  land                alias: state END\r\n"
               "  stream [secs]       stream altitude at 20 Hz (default 10 s)\r\n");

    } else if (strcasecmp(cmd, "status") == 0) {
        FlightState fs{};
        xQueuePeek(g_state_q, &fs, 0);
        static const char* names[] = {"PAD","BOOST","COAST","APOGEE","RECOVERY","END"};
        printf("state=%s  agl=%.2fm  climb=%.2fm/s  cam=%s\r\n",
               names[(int)fs.state],
               (double)fs.altitude_agl,
               (double)fs.climb_ms,
               fs.camera_on ? "ON" : "OFF");

    } else if ((strcasecmp(cmd, "state") == 0 || strcasecmp(cmd, "arm") == 0 ||
                strcasecmp(cmd, "land") == 0)) {
        // Resolve the target state
        const char* state_name = arg1;
        if (strcasecmp(cmd, "arm")  == 0) state_name = "BOOST";
        if (strcasecmp(cmd, "land") == 0) state_name = "END";
        if (!state_name) { printf("usage: state <name>\r\n"); return; }

        state_t s;
        if      (strcasecmp(state_name,"PAD")==0)      s = PAD;
        else if (strcasecmp(state_name,"BOOST")==0)    s = BOOST;
        else if (strcasecmp(state_name,"COAST")==0)    s = COAST;
        else if (strcasecmp(state_name,"APOGEE")==0)   s = APOGEE;
        else if (strcasecmp(state_name,"RECOVERY")==0) s = RECOVERY;
        else if (strcasecmp(state_name,"END")==0)      s = END;
        else { printf("unknown state '%s'\r\n", state_name); return; }

        xQueueOverwrite(g_forced_state_q, &s);
        // Wake flight_task immediately
        if (g_flight_task_handle) xTaskNotify(g_flight_task_handle, 0, eNoAction);
        printf("state=%s\r\n", state_name);

    } else if (strcasecmp(cmd, "vtxfreq") == 0 && arg1) {
        int freq = atoi(arg1);
        if (freq >= 3000 && freq <= 5900) {
            g_vtx.set_frequency((uint16_t)freq);
            printf("freq=%d\r\n", freq);
        } else {
            printf("freq must be 3000-5900 MHz\r\n");
        }

    } else if (strcasecmp(cmd, "vtxband") == 0 && arg1 && arg2) {
        VtxBand band = (strcasecmp(arg1,"A")==0) ? VtxBand::A : VtxBand::B;
        int ch = atoi(arg2);
        if (ch >= 1 && ch <= 8) {
            uint16_t freq = vtx_frequency(band, static_cast<VtxChannel>(ch-1));
            g_vtx.set_frequency(freq);
            printf("band=%s ch=%d freq=%d\r\n", arg1, ch, (int)freq);
        } else {
            printf("ch must be 1-8\r\n");
        }

    } else if (strcasecmp(cmd, "vtxpower") == 0 && arg1) {
        int mw = atoi(arg1);
        VtxPower p;
        if      (mw==0)    p = VtxPower::PIT;
        else if (mw==25)   p = VtxPower::MW25;
        else if (mw==200)  p = VtxPower::MW200;
        else if (mw==1000) p = VtxPower::MW1000;
        else if (mw==4000) p = VtxPower::MW4000;
        else { printf("power: 0 25 200 1000 4000\r\n"); return; }
        g_vtx.set_power(vtx_power_mw(p));
        printf("power=%dmW\r\n", mw);

    } else if (strcasecmp(cmd, "vtxrf") == 0 && arg1) {
        bool on = strcasecmp(arg1,"on") == 0;
        g_vtx.set_active(on);
        printf("vtxrf=%s\r\n", on ? "on" : "off");

    } else if (strcasecmp(cmd, "vtxstatus") == 0) {
        TrampStatus s{};
        if (g_vtx.get_config(s))
            printf("freq=%u  conf=%umW  actual=%umW  pit=%s\r\n",
                   s.frequency, s.conf_power, s.actual_power,
                   s.pit_mode ? "yes" : "no");
        else
            printf("no response from VTX\r\n");

    } else if (strcasecmp(cmd, "cam") == 0) {
        camera_pulse_from_task();
        printf("camera toggle requested\r\n");

    } else if (strcasecmp(cmd, "stream") == 0) {
        int secs = arg1 ? atoi(arg1) : 10;
        if (secs <= 0 || secs > 300) secs = 10;
        const float ground = get_ground_altitude();
        printf("streaming altitude for %d s (Ctrl+C to abort via land)\r\n", secs);
        const int samples = secs * 20;
        for (int i = 0; i < samples; i++) {
            float raw = get_altitude_raw();
            float agl = raw - ground;
            printf("alt=%.2f agl=%.2f\r\n", (double)raw, (double)agl);
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        printf("stream done\r\n");

    } else {
        printf("unknown command '%s'  (try 'help')\r\n", cmd);
    }
}

// ---------------------------------------------------------------------------
// usb_task — blocks on getchar(), yielding to scheduler between characters
// ---------------------------------------------------------------------------

void usb_task(void*) {
    // Wait for USB CDC enumeration
    while (!stdio_usb_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    printf("\r\n=== rocket-cam debug console ===\r\n");
    printf("type 'help' for commands\r\n> ");
    fflush(stdout);

    static char   buf[128];
    static size_t len = 0;

    while (true) {
        // Blocks here, yielding to the scheduler — no busy poll, USB stack
        // is driven by interrupt and stdio_usb callbacks.
        int c = getchar();
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (c == '\r' || c == '\n') {
            putchar('\r'); putchar('\n');
            if (len > 0) {
                buf[len] = '\0';
                _dbg_dispatch(buf);
                len = 0;
            }
            printf("> ");
            fflush(stdout);

        } else if ((c == 127 || c == '\b') && len > 0) {
            len--;
            putchar('\b'); putchar(' '); putchar('\b');
            fflush(stdout);

        } else if (c >= 0x20 && len < sizeof(buf) - 1) {
            buf[len++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}
