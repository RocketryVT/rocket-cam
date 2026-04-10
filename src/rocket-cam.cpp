#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <math.h>

#include "altimeter.hpp"
#include "irc_tramp.hpp"
#include "osd.hpp"
#include "shared.hpp"   // FlightState, NOTIF_*, queue externs

// ---------------------------------------------------------------------------
// Queue and task handle definitions (declared extern in shared.hpp)
// ---------------------------------------------------------------------------

static StaticQueue_t s_state_q_buf;
static uint8_t       s_state_q_storage[sizeof(FlightState)];
static StaticQueue_t s_forced_state_q_buf;
static uint8_t       s_forced_state_q_storage[sizeof(state_t)];

QueueHandle_t g_state_q;
QueueHandle_t g_forced_state_q;
TaskHandle_t  g_flight_task_handle;

// ---------------------------------------------------------------------------
// Peripheral objects (extern'd in debug.hpp)
// ---------------------------------------------------------------------------

static altimeter g_altimeter(ALT_I2C_INST, ALT_I2C_ADDR);
IrcTramp         g_vtx(VTX_PIO, PIN_VTX);
Osd              g_osd(OSD_UART, PIN_OSD_TX, PIN_OSD_RX);

static float s_ground_altitude = 0.0f;

float get_altitude_raw()    { return g_altimeter.get_altitude_converted(); }
float get_ground_altitude() { return s_ground_altitude; }

// ---------------------------------------------------------------------------
// Camera — pin is asserted HIGH then a one-shot timer pulls it LOW so
// flight_task is never blocked waiting for the hold period.
// ---------------------------------------------------------------------------

static bool          s_camera_on = false;
static StaticTimer_t s_cam_timer_buf;
static TimerHandle_t s_cam_timer = nullptr;

static void cam_pin_low_cb(TimerHandle_t) {
    gpio_put(PIN_CAM, 0);
}

static void camera_start() {
    if (s_camera_on) return;
    s_camera_on = true;
    gpio_put(PIN_CAM, 1);
    xTimerChangePeriod(s_cam_timer, pdMS_TO_TICKS(5000), 0);
    xTimerStart(s_cam_timer, 0);
}

static void camera_stop() {
    if (!s_camera_on) return;
    s_camera_on = false;
    gpio_put(PIN_CAM, 1);
    xTimerChangePeriod(s_cam_timer, pdMS_TO_TICKS(1000), 0);
    xTimerStart(s_cam_timer, 0);
}

// Called from usb_task — sends bit-1 notification so flight_task does the pulse.
void camera_pulse_from_task() {
    if (g_flight_task_handle) xTaskNotify(g_flight_task_handle, NOTIF_CAM, eSetBits);
}

// ---------------------------------------------------------------------------
// VTX init — runs inside vtx_task so blocking delays are fine
// ---------------------------------------------------------------------------

static void vtx_init() {
    g_vtx.begin();
    vTaskDelay(pdMS_TO_TICKS(500));

    const uint16_t freq  = vtx_frequency(VTX_BOOT_BAND, VTX_BOOT_CHANNEL);
    const uint16_t power = vtx_power_mw(VTX_BOOT_POWER);

    for (int attempt = 0; attempt < 3; attempt++) {
        TrampRFLimits limits;
        if (!g_vtx.init_rf(limits)) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        g_vtx.set_frequency(freq);
        vTaskDelay(pdMS_TO_TICKS(200));
        g_vtx.set_power(power);
        vTaskDelay(pdMS_TO_TICKS(200));
        g_vtx.set_active(true);
        return;
    }
    // VTX unresponsive — continue without it
}

// ---------------------------------------------------------------------------
// Pad-launch GPIO interrupt — wakes flight_task via notification bit 0
// ---------------------------------------------------------------------------

static StaticTask_t  s_flight_tcb;
static StackType_t   s_flight_stack[1024];

static StaticTask_t  s_vtx_tcb;
static StackType_t   s_vtx_stack[512];

static TaskHandle_t  s_heartbeat_task_handle = nullptr;
static StaticTask_t  s_heartbeat_tcb;
static StackType_t   s_heartbeat_stack[256];

static StaticTask_t  s_usb_tcb;
static StackType_t   s_usb_stack[1024];

static void pad_callback(uint /*gpio*/, uint32_t /*event_mask*/) {
    g_altimeter.unset_threshold_altitude(PIN_ALT_INT1);
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(g_flight_task_handle, NOTIF_LAUNCH, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}

// ---------------------------------------------------------------------------
// Hard-timeout timer callback — runs in timer-daemon task context (not ISR).
// Posts END state to flight_task via g_forced_state_q.
// ---------------------------------------------------------------------------

static void end_timer_cb(TimerHandle_t /*t*/) {
    state_t s = END;
    xQueueOverwrite(g_forced_state_q, &s);
    // Also notify flight_task so it wakes immediately
    if (g_flight_task_handle) {
        xTaskNotify(g_flight_task_handle, 0, eNoAction);
    }
}

// ---------------------------------------------------------------------------
// flight_task  (priority 3, core 0)
// ---------------------------------------------------------------------------

static void flight_task(void*) {
    // Hardware init
    i2c_init(ALT_I2C_INST, ALT_I2C_BAUD);
    gpio_set_function(PIN_ALT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_ALT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_ALT_SDA);
    gpio_pull_up(PIN_ALT_SCL);

    gpio_init(PIN_ALT_INT1);
    gpio_pull_up(PIN_ALT_INT1);

    gpio_init(PIN_CAM);
    gpio_set_dir(PIN_CAM, GPIO_OUT);
    gpio_put(PIN_CAM, 0);

    g_altimeter.initialize();
    s_ground_altitude          = g_altimeter.get_altitude_converted();
    const float ground_altitude = s_ground_altitude;
    float       prev_altitude   = ground_altitude;

    g_altimeter.set_threshold_altitude(
        ground_altitude + LAUNCH_THRESHOLD_M, PIN_ALT_INT1, &pad_callback);

    g_osd.begin();

    state_t       state        = PAD;
    uint32_t      static_count = 0;
    TickType_t    last_tick    = xTaskGetTickCount();
    TimerHandle_t end_timer    = nullptr;
    bool          ended        = false;

    // Publish initial state
    FlightState fs{PAD, 0.0f, 0.0f, s_camera_on};
    xQueueOverwrite(g_state_q, &fs);

    while (true) {
        // Block up to 1 s; any notification wakes us early.
        uint32_t notif = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notif, pdMS_TO_TICKS(1000));

        // Camera toggle requested by usb_task
        if (notif & NOTIF_CAM) {
            if (s_camera_on) camera_stop();
            else             camera_start();
        }

        // Snapshot state before any transitions this iteration
        const state_t prev_state = state;

        // Accept a forced state from usb_task or end_timer_cb
        state_t forced;
        if (xQueueReceive(g_forced_state_q, &forced, 0) == pdTRUE) {
            state = forced;
            static_count = 0;
        }

        // Altimeter read + dt
        const TickType_t now = xTaskGetTickCount();
        const float dt_s = (float)(now - last_tick) / (float)configTICK_RATE_HZ;
        last_tick = now;

        const float altitude = g_altimeter.get_altitude_converted();
        const float climb    = (dt_s > 0.0f) ? (altitude - prev_altitude) / dt_s : 0.0f;
        const float alt_agl  = altitude - ground_altitude;
        prev_altitude = altitude;

        // State transitions

        if (state == PAD && ((notif & NOTIF_LAUNCH) || alt_agl >= 40.0f)) {
            state = BOOST;
            end_timer = xTimerCreate("end", pdMS_TO_TICKS(FLIGHT_TIMEOUT_MS),
                                     pdFALSE, nullptr, end_timer_cb);
            if (end_timer) xTimerStart(end_timer, 0);

        } else if (state != PAD && state != END) {
            if (fabsf(climb) <= LANDING_THRESHOLD_M) {
                static_count++;
            } else {
                static_count = 0;
            }
            if (static_count >= LANDING_COUNT) {
                state = END;
                if (end_timer) xTimerStop(end_timer, 0);
            }
        }

        // Start recording and go full power on first exit from PAD
        if (prev_state == PAD && state != PAD) {
            g_vtx.set_power(vtx_power_mw(VtxPower::MW4000));
            camera_start();
            
        }

        // On first entry to END: stop camera and put VTX in pit mode
        if (state == END && !ended) {
            ended = true;
            g_vtx.set_power(vtx_power_mw(VtxPower::PIT));
            camera_stop();
        }

        // OSD update
        const char* state_str;
        switch (state) {
            case BOOST:    state_str = "BOOST";    break;
            case COAST:    state_str = "COAST";    break;
            case APOGEE:   state_str = "APOGEE";   break;
            case RECOVERY: state_str = "RECOVERY"; break;
            case END:      state_str = "END";      break;
            default:       state_str = "PAD";      break;
        }
        char vtx_line[50], stage_line[50];
        snprintf(vtx_line,   sizeof(vtx_line),   "BAND A CH 1 4000MW");
        snprintf(stage_line, sizeof(stage_line),  "FLIGHT STAGE: %s", state_str);
        g_osd.update(alt_agl, climb, (uint32_t)state, vtx_line, stage_line);

        // Publish snapshot
        FlightState pub{state, alt_agl, climb, s_camera_on};
        xQueueOverwrite(g_state_q, &pub);

        if (state == END) {
            vTaskSuspend(nullptr);
        }
    }
}

// ---------------------------------------------------------------------------
// vtx_task  (priority 2, core 0)
// ---------------------------------------------------------------------------

static void vtx_task(void*) {
    vtx_init();
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// heartbeat_task  (priority 1, core 1)
// ---------------------------------------------------------------------------

static void heartbeat_task(void*) {
    // PAD:   short double-blink  1 0 1 0 0
    // BOOST: mostly on           1 1 1 1 0
    // END:   slow single blink   0 0 0 0 1
    static const bool seq_pad[5]   = {true,  false, true,  false, false};
    static const bool seq_boost[5] = {true,  true,  true,  true,  false};
    static const bool seq_end[5]   = {false, false, false, false, true};

    const TickType_t period = pdMS_TO_TICKS(1000 / HEARTBEAT_HZ);
    uint8_t     idx = 0;
    FlightState fs{PAD, 0.0f, 0.0f, false};

    while (true) {
        xQueuePeek(g_state_q, &fs, 0);

        bool on;
        if      (fs.state == PAD) on = seq_pad[idx];
        else if (fs.state == END) on = seq_end[idx];
        else                      on = seq_boost[idx];

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
        idx = (idx + 1) % 5;
        vTaskDelay(period);
    }
}

// ---------------------------------------------------------------------------
// usb_task — body defined in debug.hpp, forward-declared here
// ---------------------------------------------------------------------------

#include "debug.hpp"

// ---------------------------------------------------------------------------
// Static memory for FreeRTOS internal tasks (required by
// configSUPPORT_STATIC_ALLOCATION=1)
// ---------------------------------------------------------------------------

extern "C" {

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t**  ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE* puxIdleTaskStackSize) {
    static StaticTask_t idle_tcb;
    static StackType_t  idle_stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer   = &idle_tcb;
    *ppxIdleTaskStackBuffer = idle_stack;
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

// RP2040 SMP: one passive idle task per core
void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                          StackType_t**  ppxIdleTaskStackBuffer,
                                          configSTACK_DEPTH_TYPE* puxIdleTaskStackSize,
                                          BaseType_t xPassiveIdleTaskIndex) {
    static StaticTask_t passive_tcb[configNUMBER_OF_CORES - 1];
    static StackType_t  passive_stack[configNUMBER_OF_CORES - 1][configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer   = &passive_tcb[xPassiveIdleTaskIndex];
    *ppxIdleTaskStackBuffer = passive_stack[xPassiveIdleTaskIndex];
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t**  ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE* puxTimerTaskStackSize) {
    static StaticTask_t timer_tcb;
    static StackType_t  timer_stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer   = &timer_tcb;
    *ppxTimerTaskStackBuffer = timer_stack;
    *puxTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    stdio_init_all();
    cyw43_arch_init();

    g_state_q        = xQueueCreateStatic(1, sizeof(FlightState),
                                          s_state_q_storage, &s_state_q_buf);
    g_forced_state_q = xQueueCreateStatic(1, sizeof(state_t),
                                          s_forced_state_q_storage, &s_forced_state_q_buf);

    s_cam_timer = xTimerCreateStatic("cam_pin", pdMS_TO_TICKS(1000),
                                     pdFALSE, nullptr, cam_pin_low_cb,
                                     &s_cam_timer_buf);

    g_flight_task_handle =
        xTaskCreateStatic(flight_task,    "flight",    1024, nullptr, 3,
                          s_flight_stack,    &s_flight_tcb);
    xTaskCreateStatic(vtx_task,       "vtx",       512,  nullptr, 2,
                      s_vtx_stack,       &s_vtx_tcb);
    s_heartbeat_task_handle =
        xTaskCreateStatic(heartbeat_task, "heartbeat", 256,  nullptr, 1,
                          s_heartbeat_stack, &s_heartbeat_tcb);
    xTaskCreateStatic(usb_task,       "usb",       1024, nullptr, 1,
                      s_usb_stack,       &s_usb_tcb);

    // Pin heartbeat to core 1; all others run on core 0 by default
    vTaskCoreAffinitySet(s_heartbeat_task_handle, 1u << 1);

    vTaskStartScheduler();
    for (;;) {}
}
