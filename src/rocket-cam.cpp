#include "cyw43_configport.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "pico/cyw43_arch.h"
#include <cstdio>
#include <math.h>

#include "altimeter.hpp"
#include "irc_tramp.hpp"
#include "osd.hpp"
#include "pins.hpp"

#define DEBUG_USB 

#if defined(DEBUG_USB)
    #include "debug.hpp"
#endif


void pad_callback(uint gpio, uint32_t event_mask);
int64_t set_altitude_callback(alarm_id_t id, void* user_data);
int64_t toggle_camera_callback(alarm_id_t id, void* user_data);
int64_t end_timeout_callback(alarm_id_t id, void* user_data);
bool changing_altitude_callback(repeating_timer_t *rt);
bool heartbeat_callback(repeating_timer_t *rt);
void heartbeat_core();
void vtx_init();
const char* state_name(state_t current_state);

volatile float     altitude       = 0.0f;
volatile float     prev_altitude  = 0.0f;
volatile float     ground_altitude = 0.0f;
volatile state_t   state          = PAD;
volatile uint8_t   led_counter    = 0;
volatile bool      enabled_camera = false;

repeating_timer_t data_timer;
repeating_timer_t heartbeat_timer;
volatile alarm_id_t end_timer;

altimeter altimeter(ALT_I2C_INST, ALT_I2C_ADDR);
IrcTramp  vtx(VTX_PIO, PIN_VTX);
Osd       osd(OSD_UART, PIN_OSD_TX, PIN_OSD_RX);

int main() {
    cyw43_arch_init();
    #if defined(DEBUG_USB)
    debug_init();
    #endif

    // Altimeter I2C
    i2c_init(ALT_I2C_INST, ALT_I2C_BAUD);
    gpio_set_function(PIN_ALT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_ALT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_ALT_SDA);
    gpio_pull_up(PIN_ALT_SCL);

    gpio_init(PIN_ALT_INT1);
    gpio_pull_up(PIN_ALT_INT1);

    // Camera trigger pin
    gpio_init(PIN_CAM);
    gpio_set_dir(PIN_CAM, GPIO_OUT);
    gpio_pull_down(PIN_CAM);
    gpio_put(PIN_CAM, 0);

    alarm_pool_init_default();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    altimeter.initialize();
    ground_altitude = altimeter.get_altitude_converted();
    prev_altitude = ground_altitude;
    altimeter.set_threshold_altitude(
        ground_altitude + LAUNCH_THRESHOLD_M, PIN_ALT_INT1, &pad_callback);

    vtx_init();
    osd.begin();
    add_repeating_timer_ms(1000, changing_altitude_callback, NULL, &data_timer);

    sleep_ms(5000);
    toggle_camera_callback(0, NULL);

    multicore_launch_core1(heartbeat_core);

#if defined(DEBUG_USB)
    while (1) { debug_poll(); }
#else
    while (1) { tight_loop_contents(); }
#endif
}

// Set VTX to boot frequency and power, RF on. Retries 3x if no response.
void vtx_init() {
    vtx.begin();
    sleep_ms(500);  // VTX power-on settling

    const uint16_t freq  = vtx_frequency(VtxBand::A, VtxChannel::CH1);
    const uint16_t power = vtx_power_mw(VtxPower::MW4000);

    for (int attempt = 0; attempt < 3; attempt++) {
        TrampRFLimits limits;
        if (!vtx.init_rf(limits)) {
            sleep_ms(200);
            continue;
        }
        vtx.set_frequency(freq);
        sleep_ms(200);
        vtx.set_power(power);
        sleep_ms(200);
        vtx.set_active(true);
        return;
    }
    // VTX unresponsive — continue without it
}

void pad_callback(uint gpio, uint32_t event_mask) {
    altimeter.unset_threshold_altitude(PIN_ALT_INT1);
    state = BOOST;
    add_alarm_in_ms(1000, &set_altitude_callback, NULL, true);
}

int64_t set_altitude_callback(alarm_id_t id, void* user_data) {
    end_timer = add_alarm_in_ms(FLIGHT_TIMEOUT_MS, end_timeout_callback, NULL, true);
    return 0;
}

int64_t end_timeout_callback(alarm_id_t, void* user_data) {
    cancel_repeating_timer(&data_timer);
    state = END;
    toggle_camera_callback(0, NULL);
    vtx.set_active(false);  // pit mode on landing
    return 0;
}

// =============================================================================
// Heartbeat (core 1)
// =============================================================================

void heartbeat_core() {
    add_repeating_timer_us(
        -1000000 / HEARTBEAT_HZ, &heartbeat_callback, NULL, &heartbeat_timer);
    while (1) { tight_loop_contents(); }
}

bool heartbeat_callback(repeating_timer_t *rt) {
    // PAD:  short double-blink
    // BOOST: solid on
    // END:  slow single blink
    const bool seq_pad[]   = {true, false, true, false, false};
    const bool seq_boost[] = {true, true,  true, true,  false};
    const bool seq_end[]   = {false, false, false, false, true};

    bool on;
    if (state == PAD)        on = seq_pad[led_counter];
    else if (state == BOOST) on = seq_boost[led_counter];
    else                     on = seq_end[led_counter];

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    led_counter = (led_counter + 1) % 5;
    return true;
}

// =============================================================================
// Camera toggle
// =============================================================================

int64_t toggle_camera_callback(alarm_id_t id, void* user_data) {
    static bool toggled = false;
    gpio_put(PIN_CAM, !toggled);
    if (!toggled) {
        uint32_t delay = enabled_camera ? 1000 : 5000;
        add_alarm_in_ms(delay, toggle_camera_callback, NULL, true);
        toggled = true;
        return 0;
    }
    enabled_camera = !enabled_camera;
    toggled = false;
    return 0;
}

const char* state_name(state_t current_state) {
    switch (current_state) {
    case PAD:
        return "PAD";
    case BOOST:
        return "BOOST";
    case COAST:
        return "COAST";
    case APOGEE:
        return "APOGEE";
    case RECOVERY:
        return "RECOVERY";
    case END:
        return "END";
    default:
        return "UNKNOWN";
    }
}

// =============================================================================
// Altitude monitoring (1 Hz)
// =============================================================================

bool changing_altitude_callback(repeating_timer_t *rt) {
    static uint32_t static_counter = 0;
    char vtx_status[50];
    char stage_status[50];

    altitude       = altimeter.get_altitude_converted();
    float climb    = altitude - prev_altitude;  // m/s (fires at 1 Hz)
    float alt_agl  = altitude - ground_altitude;

    snprintf(vtx_status, sizeof(vtx_status), "BAND A CH 1 4000MW");
    snprintf(stage_status, sizeof(stage_status), "FLIGHT STAGE: %s", state_name(state));

    osd.update(alt_agl, climb, static_cast<uint32_t>(state), vtx_status, stage_status);

    if (state != PAD && fabsf(climb) <= LANDING_THRESHOLD_M) {
        static_counter++;
    } else {
        static_counter = 0;
    }

    if (static_counter >= LANDING_COUNT) {
        state = END;
        cancel_repeating_timer(&data_timer);
        cancel_alarm(end_timer);
        toggle_camera_callback(0, NULL);
        vtx.set_active(false);  // pit mode on landing
    }

    prev_altitude = altitude;
    return true;
}
