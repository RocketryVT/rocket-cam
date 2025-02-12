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

#define MOTOR_BURN_TIME 3900 /* (M2500T Burn in ms) */

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
int64_t boost_callback(alarm_id_t id, void* user_data);
int64_t apogee_callback(alarm_id_t id, void* user_data);
int64_t coast_callback(alarm_id_t id, void* user_data);
void recovery_callback(uint gpio, uint32_t event_mask);

bool timer_callback(repeating_timer_t *rt);

bool heartbeat_callback(repeating_timer_t *rt);
void heartbeat_core();

volatile float altitude = 0.0f;
volatile float prev_altitude = 0.0f;
volatile float velocity = 0.0f;
volatile state_t state = PAD;
volatile float threshold_altitude = 30.0f;
volatile float threshold_velocity = 30.0f;

volatile float moving_average[MOVING_AVG_MAX_SIZE];
volatile uint8_t moving_average_offset = 0;
volatile uint8_t moving_average_size = 0;
volatile float moving_average_sum = 0;

volatile uint8_t led_counter;

repeating_timer_t data_timer;
repeating_timer_t heartbeat_timer;

float ground_altitude = 0.0f;

altimeter altimeter(i2c1, MPL3115A2_ADDR);

uint8_t *altimeter_buffer;

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
    gpio_put(CAM_PIN, false);
    
    alarm_pool_init_default();

    altimeter.initialize(30.0f, INT1_PIN, &pad_callback);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    ground_altitude = altimeter.get_altitude_converted();
    prev_altitude = ground_altitude;

    altimeter.expose_buffer(&altimeter_buffer);

    add_repeating_timer_us(-1000000 / DATA_RATE_HZ,  &timer_callback, NULL, &data_timer);

    multicore_launch_core1(heartbeat_core);

    while (1) {
        tight_loop_contents();
    }
}

// PRIMARY THREAD RELATED FUNCTIONS AND CALLBACKS
//===============================================================================

bool timer_callback(repeating_timer_t *rt) {
    if (moving_average_size == MOVING_AVG_MAX_SIZE) {
        moving_average_sum -= moving_average[moving_average_offset];
    } else {
        moving_average_size++;
    }

    moving_average[moving_average_offset] = altimeter.get_altitude_converted();
    moving_average_sum += moving_average[moving_average_offset];
    moving_average_offset = (moving_average_offset + 1) % MOVING_AVG_MAX_SIZE;

    prev_altitude = altitude;
    altitude = moving_average_sum / moving_average_size;

    velocity = (altitude - prev_altitude) / 0.01f;

    return true;
}

/**
 * @brief Call back function for when rocket is on the pad
 * 
 * @param gpio pin number of interrupt
 * @param event_mask interrupt condition, value is set by PICO_SDK
 *  GPIO_IRQ_LEVEL_LOW = 0x1u,
 *  GPIO_IRQ_LEVEL_HIGH = 0x2u,
 *  GPIO_IRQ_EDGE_FALL = 0x4u,
 *  GPIO_IRQ_EDGE_RISE = 0x8u,
 * @link https://www.raspberrypi.com/documentation/pico-sdk/hardware/gpio.html#ga6347e27da3ab34f1ea65b5ae16ab724f
 */
void pad_callback(uint gpio, uint32_t event_mask) {
    altimeter.unset_threshold_altitude(INT1_PIN);
    state = BOOST;
    gpio_put(CAM_PIN, true);
    sleep_ms(10);
    gpio_put(CAM_PIN, false);
    // start motor burn timer with boost transition function as callback
    add_alarm_in_ms(MOTOR_BURN_TIME, &boost_callback, NULL, false);
}

int64_t boost_callback(alarm_id_t id, void* user_data) {
    // Configure accelerometer and/or altimeter to generate interrupt
    // for when velocity is negative with this function as callback to
    // transition to APOGEE
    state = COAST;
    add_alarm_in_ms(1000, &coast_callback, NULL, false);
    return 0;
}

int64_t coast_callback(alarm_id_t id, void* user_data) {
    // Want to somehow immediately transition to RECOVERY from APOGEE (extremely short timer?)
    if (velocity <= 0.0f) {
        state = APOGEE;
        add_alarm_in_ms(1, &apogee_callback, NULL, false);
    } else {
        add_alarm_in_ms(250, &coast_callback, NULL, false);
    }
    return 0;
}

int64_t apogee_callback(alarm_id_t id, void* user_data) {
    state = RECOVERY;
    // Set altimeter interrupt to occur for when rocket touches back to the ground
    altimeter.set_threshold_altitude((ground_altitude + 10.0f), INT1_PIN, &recovery_callback);

    return 0;
}

void recovery_callback(uint gpio, uint32_t event_mask) {
    // Essentially just a signal to stop logging data
    gpio_put(CAM_PIN, true);
    sleep_ms(10);
    gpio_put(CAM_PIN, false);
    state = END;
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
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    bool led_status = sequence[led_counter];
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}

