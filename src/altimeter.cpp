#include "altimeter.hpp"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

altimeter::altimeter(i2c_inst_t* inst, uint8_t addr)
    : inst(inst), addr(addr) {}

void altimeter::initialize() {
    // Active mode, OSR=16, altimeter mode (register 0x26 = 0x89 for RP2xxx timing)
    buffer[0] = 0x26;
    buffer[1] = 0x89;
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Allow sensor to take its first measurement
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Spin until we get a non-zero reading, yielding between attempts
    float alt = 0.0f;
    while (alt == 0.0f) {
        alt = get_altitude_converted();
        if (alt == 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void altimeter::initialize(float threshold_altitude, uint8_t interrupt_pin,
                           gpio_irq_callback_t callback) {
    initialize();
    vTaskDelay(pdMS_TO_TICKS(1000));

    float alt = 0.0f;
    while (alt == 0.0f) {
        alt = get_altitude_converted();
        if (alt == 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    threshold_altitude += alt;
    _arm_interrupt(threshold_altitude, interrupt_pin, callback);
}

void altimeter::set_threshold_altitude(float threshold_altitude,
                                       uint8_t interrupt_pin,
                                       gpio_irq_callback_t callback) {
    _arm_interrupt(threshold_altitude, interrupt_pin, callback);
}

void altimeter::_arm_interrupt(float threshold_altitude, uint8_t interrupt_pin,
                               gpio_irq_callback_t callback) {
    // INT pins active-low, internal pull-ups enabled (register 0x28)
    buffer[0] = 0x28;
    buffer[1] = 0x01;
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Altitude target MSB (register 0x16)
    buffer[0] = 0x16;
    buffer[1] = (uint8_t)(((int16_t)threshold_altitude) >> 8);
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Altitude target LSB (register 0x17)
    buffer[0] = 0x17;
    buffer[1] = (uint8_t)((int16_t)threshold_altitude);
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Enable altitude-threshold interrupt (register 0x29, bit 3)
    buffer[0] = 0x29;
    buffer[1] = 0x08;
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Route threshold interrupt to INT1 pin (register 0x2A, bit 3)
    buffer[0] = 0x2A;
    buffer[1] = 0x08;
    i2c_write_blocking(inst, addr, buffer, 2, true);

    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_EDGE_FALL,
                                       true, callback);
}

void altimeter::unset_threshold_altitude(uint8_t interrupt_pin) {
    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_EDGE_FALL,
                                       false, nullptr);

    // Disable altitude-threshold interrupt
    buffer[0] = 0x29;
    buffer[1] = 0x00;
    i2c_write_blocking(inst, addr, buffer, 2, true);

    // Clear INT1 routing
    buffer[0] = 0x2A;
    buffer[1] = 0x00;
    i2c_write_blocking(inst, addr, buffer, 2, true);
}

float altimeter::get_altitude_converted() {
    uint8_t reg = 0x01;
    i2c_write_blocking(inst, addr, &reg, 1, true);
    i2c_read_blocking(inst, addr, altitude_buffer, 4, false);
    // MPL3115A2 datasheet: integer part in [0..1], fractional in [2] bits [7:4]
    float alt = (float)((int16_t)((altitude_buffer[0] << 8) | altitude_buffer[1]))
              + (float)(altitude_buffer[2] >> 4) * 0.0625f;
    return alt;
}

void altimeter::get_altitude_raw(uint8_t* out) {
    uint8_t reg = 0x01;
    i2c_write_blocking(inst, addr, &reg, 1, true);
    i2c_read_blocking(inst, addr, out, 3, false);
}

uint32_t altimeter::expose_buffer(uint8_t** out) {
    *out = altitude_buffer;
    return sizeof(altitude_buffer);
}
