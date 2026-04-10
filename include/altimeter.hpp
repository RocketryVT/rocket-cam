#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <cstdint>

class altimeter {
public:
    altimeter(i2c_inst_t* inst, uint8_t addr);

    // Configure the sensor for continuous altimeter mode and wait for a valid
    // first reading. Must be called from a FreeRTOS task (uses vTaskDelay).
    void initialize();

    // As above, then arm the launch-detect interrupt on interrupt_pin.
    void initialize(float threshold_altitude, uint8_t interrupt_pin,
                    gpio_irq_callback_t callback);

    // Arm the altitude-threshold interrupt without re-initialising the sensor.
    void set_threshold_altitude(float threshold_altitude, uint8_t interrupt_pin,
                                gpio_irq_callback_t callback);

    // Disarm the altitude-threshold interrupt.
    void unset_threshold_altitude(uint8_t interrupt_pin);

    // Read and return the current altitude in metres (MPL3115A2 fixed-point).
    float get_altitude_converted();

    void     get_altitude_raw(uint8_t* buffer);
    uint32_t expose_buffer(uint8_t** buffer);

private:
    uint8_t      altitude_buffer[4];
    uint8_t      buffer[4];
    uint8_t      addr;
    i2c_inst_t*  inst;

    void _arm_interrupt(float threshold_altitude, uint8_t interrupt_pin,
                        gpio_irq_callback_t callback);
};
