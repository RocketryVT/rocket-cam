#pragma once

#include <stdint.h>

// =============================================================================
// Hardware pins
// =============================================================================

// Onboard LED (via CYW43 on Pico W — not a GPIO)
// Accessed through cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ...)

// Altimeter (MPL3115A2) — I2C1
const uint8_t PIN_ALT_SDA   = 14;
const uint8_t PIN_ALT_SCL   = 15;
const uint8_t PIN_ALT_INT1  = 13;  // INT1 interrupt output from sensor

// Camera trigger (Runcam Mini DVR)
const uint8_t PIN_CAM       = 0;   // Pull high to start/stop recording

// VTX (RushFPV 3.3GHz 4W) — single-wire half-duplex IRC Tramp via PIO
const uint8_t PIN_VTX       = 4;

// OSD (Holybro Micro OSD V2) — full-duplex UART
const uint8_t PIN_OSD_TX    = 0;   // uart0 TX
const uint8_t PIN_OSD_RX    = 1;   // uart0 RX

// =============================================================================
// Peripheral configuration
// =============================================================================

// Altimeter I2C
#define ALT_I2C_INST        i2c1
#define ALT_I2C_BAUD        400000
#define ALT_I2C_ADDR        0x60   // MPL3115A2

// Camera
#define CAM_I2C_INST        i2c1   // shared bus with altimeter

// VTX PIO
#define VTX_PIO             pio0

// OSD UART
#define OSD_UART            uart0
#define OSD_BAUD            57600

// =============================================================================
// Application timing
// =============================================================================

#define HEARTBEAT_HZ        5      // LED heartbeat rate

// Altitude must not change by more than this (m) per second for LANDING_COUNT
// consecutive seconds before we declare landing
#define LANDING_THRESHOLD_M 1.0f
#define LANDING_COUNT       30

// Arm threshold: altitude must rise this many metres above ground to detect launch
#define LAUNCH_THRESHOLD_M  30.0f

// Hard timeout after launch — declare END regardless of altitude (30 min)
#define FLIGHT_TIMEOUT_MS   1800000

// =============================================================================
// VTX frequency and power tables (RushFPV 3.3GHz 4W — from datasheet)
// =============================================================================

enum class VtxBand : uint8_t {
    A = 0,  // 3G3 BAND A: 3330–3470 MHz
    B = 1,  // 3G3 BAND B: 3170–3310 MHz
};

enum class VtxChannel : uint8_t {
    CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8,
};

enum class VtxPower : uint8_t {
    PIT    = 0,  //    ~0 mW  (pit mode / RF off)
    MW25   = 1,  //   25 mW
    MW200  = 2,  //  200 mW
    MW1000 = 3,  // 1000 mW
    MW4000 = 4,  // 4000 mW (max)
};

// Resolve enums to wire values at compile time
constexpr uint16_t vtx_frequency(VtxBand band, VtxChannel ch) {
    constexpr uint16_t table[2][8] = {
        {3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470},  // Band A
        {3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310},  // Band B
    };
    return table[static_cast<uint8_t>(band)][static_cast<uint8_t>(ch)];
}

constexpr uint16_t vtx_power_mw(VtxPower p) {
    constexpr uint16_t table[] = {0, 25, 200, 1000, 4000};
    return table[static_cast<uint8_t>(p)];
}

// =============================================================================
// Flight state machine
// =============================================================================

typedef enum {
    PAD = 0,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

// Boot configuration
constexpr VtxBand    VTX_BOOT_BAND    = VtxBand::A;
constexpr VtxChannel VTX_BOOT_CHANNEL = VtxChannel::CH1;
constexpr VtxPower   VTX_BOOT_POWER   = VtxPower::MW4000;
constexpr VtxPower   VTX_LAND_POWER   = VtxPower::PIT;
