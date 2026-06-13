#pragma once

// Shared definitions between rocket-cam.cpp and debug.hpp.
// Include this before any file that needs FlightState or the shared queues.

#include "FreeRTOS.h"
#include "queue.h"
#include "pins.hpp"  // state_t

// ---------------------------------------------------------------------------
// Task notification bits (sent to "flight" task)
// ---------------------------------------------------------------------------
#define NOTIF_LAUNCH  (1u << 0)  // altitude threshold crossed (from ISR)
#define NOTIF_CAM     (1u << 1)  // camera toggle requested (from usb_task)

// ---------------------------------------------------------------------------
// Flight state snapshot published by flight_task at 1 Hz
// ---------------------------------------------------------------------------
struct FlightState {
    state_t state;
    float   altitude_agl;  // metres above ground
    float   climb_ms;      // m/s, positive = climbing
    bool    camera_on;
};

// ---------------------------------------------------------------------------
// BLE camera control — plain PODs so no protobuf/btstack types leak into the
// flight loop. The BLE layer (ble.cpp) decodes nanopb into a CamCmd and the
// flight loop reads it via ble_take_cam_cmd(); the loop publishes a CamCfg
// snapshot back via ble_publish_cam_cfg(). See ble.hpp.
// ---------------------------------------------------------------------------
struct CamCmd {
    // Each has_* flag mirrors the proto3 `optional` presence bit, so the flight
    // loop applies only the fields the phone actually set.
    bool     has_band;    uint8_t  band;          // VtxBand value (0=A, 1=B)
    bool     has_channel; uint8_t  channel;       // 1..8 (1-based)
    bool     has_freq;    uint16_t freq_mhz;      // explicit frequency override
    bool     has_power;   uint8_t  power;         // VtxPowerLevel (== VtxPower)
    bool     has_rf;      bool     rf_enabled;    // false = pit mode
    bool     has_record;  bool     camera_record; // start/stop recording
    bool     request_status;                      // emit a fresh CamCfg
};

struct CamCfg {
    uint8_t  band;             // VtxBand value
    uint8_t  channel;          // 1..8, 0 if freq is off-grid
    uint16_t freq_mhz;
    uint8_t  power;            // VtxPowerLevel
    uint16_t actual_power_mw;  // from VTX 'v' response, 0 if unknown
    bool     rf_enabled;
    bool     camera_recording;
    uint8_t  flight_state;     // state_t
    float    altitude_agl_m;
    bool     vtx_responsive;
};

// ---------------------------------------------------------------------------
// Queues and task handles defined in rocket-cam.cpp
// ---------------------------------------------------------------------------
extern QueueHandle_t  g_state_q;           // FlightState, depth 1, overwrite
extern QueueHandle_t  g_forced_state_q;    // state_t,     depth 1, overwrite
extern TaskHandle_t   g_flight_task_handle; // flight_task handle for notifications

// ---------------------------------------------------------------------------
// Altimeter access helper (defined in rocket-cam.cpp, for debug streaming)
// ---------------------------------------------------------------------------
float get_altitude_raw();   // absolute altitude in metres
float get_ground_altitude(); // ground reference captured at boot
