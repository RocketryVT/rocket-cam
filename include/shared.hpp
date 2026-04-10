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
