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
#include <inttypes.h>
#include <math.h>

#include "altimeter.hpp"

#define MOVING_AVG_MAX_SIZE 10
#define MPL3115A2_ADDR 0x60

#define MAX_SCL 400000
#define DATA_RATE_HZ 100
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)
#define INT1_PIN 13 // INT1 PIN on MPL3115A2 connected to GPIO13
#define CAM_SDA 14
#define CAM_SCL 15
#define CAM_PIN 0

#define HEART_RATE_HZ 5

typedef enum {
    PAD = 0,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

void pad_callback(uint gpio, uint32_t event_mask);
int64_t set_altitude_callback(alarm_id_t id, void* user_data);
void end_callback(uint gpio, uint32_t event_mask);
int64_t toggle_camera_callback(alarm_id_t id, void* user_data);
int64_t end_timeout_callback(alarm_id_t id, void* user_data);
bool changing_altitude_callback(repeating_timer_t *rt);
bool heartbeat_callback(repeating_timer_t *rt);
void heartbeat_core();

volatile float altitude = 0.0f;
volatile float prev_altitude = 0.0f;
volatile state_t state = PAD;
volatile float threshold_altitude = 30.0f;
volatile float ground_altitude = 0.0f;

volatile uint8_t led_counter;

volatile bool enabled_camera = false;

repeating_timer_t data_timer;
repeating_timer_t heartbeat_timer;

altimeter altimeter(i2c1, MPL3115A2_ADDR);

volatile alarm_id_t end_timer;

int main() {
    cyw43_arch_init();

    i2c_init(i2c1, MAX_SCL);
    gpio_set_function(CAM_SDA, GPIO_FUNC_I2C);
    gpio_set_function(CAM_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(CAM_SDA);
    gpio_pull_up(CAM_SCL);

    gpio_init(INT1_PIN);
    gpio_pull_up(INT1_PIN);

    gpio_init(CAM_PIN);
    gpio_set_dir(CAM_PIN, GPIO_OUT);
    gpio_pull_down(CAM_PIN);
    gpio_put(CAM_PIN, 0);
    
    alarm_pool_init_default();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    altimeter.initialize();

    ground_altitude = altimeter.get_altitude_converted();
    altimeter.set_threshold_altitude((ground_altitude + threshold_altitude), INT1_PIN, &pad_callback);

    sleep_ms(5000);
    toggle_camera_callback(0, NULL);

    multicore_launch_core1(heartbeat_core);

    while (1) {
        tight_loop_contents();
    }
}

void pad_callback(uint gpio, uint32_t event_mask) {
    altimeter.unset_threshold_altitude(INT1_PIN);
    state = BOOST;
    // I actually don't care what state the rocket is in for cam activity, i just need to know when it launches and when it lands
    add_alarm_in_ms(1000, &set_altitude_callback, NULL, true);
}

int64_t set_altitude_callback(alarm_id_t id, void* user_data) {
    add_repeating_timer_ms(1000, changing_altitude_callback, NULL, &data_timer);

    end_timer = add_alarm_in_ms(1800000, end_timeout_callback, NULL, true);
    return 0;
}

int64_t end_timeout_callback(alarm_id_t, void* user_data) {
    cancel_repeating_timer(&data_timer);
    state = END;
    toggle_camera_callback(0, NULL);
    return 0;
}

// HEARTBEAT THREAD
//===============================================================================

void heartbeat_core() {
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);

    while (1) {
        tight_loop_contents();
    }
}

bool heartbeat_callback(repeating_timer_t *rt) {
    const bool sequence_0[] = {true, false, true, false, false};
    const bool sequence_1[] = {true, true, true, true, false};
    const bool sequence_2[] = {false, false, false, false, true};
    const uint8_t sequence_length = 5;

    bool led_status = true;
    if (state == PAD) {
        led_status = sequence_0[led_counter];
    } else if (state == BOOST) {
        led_status = sequence_1[led_counter];
    } else if (state == END) {
        led_status = sequence_2[led_counter];
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;

    return true;
}


int64_t toggle_camera_callback(alarm_id_t id, void* user_data) {
    static bool toggled = false;
    gpio_put(CAM_PIN, !toggled);
    if (!toggled) {
        if (!enabled_camera) {
            add_alarm_in_ms(5000, toggle_camera_callback, NULL, true);
        } else {
            add_alarm_in_ms(1000, toggle_camera_callback, NULL, true);
        }
        toggled = true;
        return 0;
    }
    enabled_camera = !enabled_camera;
    toggled = false;
    return 0;
}

bool changing_altitude_callback(repeating_timer_t *rt) {
    static uint32_t static_counter = 0;
    altitude = altimeter.get_altitude_converted();

    if (fabs(altitude - prev_altitude) <= 1.f) {
        static_counter++;
    }

    if (static_counter >= 30) {
        state = END;
        cancel_repeating_timer(&data_timer);
        cancel_alarm(end_timer);
        toggle_camera_callback(0, NULL);
    }

    prev_altitude = altitude;

    return true;
}
