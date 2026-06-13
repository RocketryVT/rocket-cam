#pragma once

// BLE GATT peripheral for rocket-cam camera control.
//
// Transport: raw nanopb sigma2.Envelope per characteristic (ATT handles
// framing). The write characteristic carries Envelope{cam_command}; the
// status characteristic returns / notifies Envelope{cam_config}.
//
// Threading: BTstack runs in the cyw43 async context (threadsafe_background).
// Its ATT callbacks execute with the async-context lock held, so they cannot
// interleave with a holder of that lock. The flight loop therefore uses
// ble_take_cam_cmd() / ble_publish_cam_cfg(), which take the same lock, to
// exchange data with the BLE context without any FreeRTOS queue crossing
// contexts.

#include "shared.hpp"  // CamCmd, CamCfg

// Initialise BTstack, the GATT server, and start advertising as "RocketCam".
// Call once, after cyw43_arch_init(), from a FreeRTOS task.
void ble_init();

// Flight loop: pull a pending BLE command (returns false if none). Clears the
// pending flag. Safe to call from a normal task.
bool ble_take_cam_cmd(CamCmd* out);

// Flight loop: publish the latest device config so BLE reads/notifications
// reflect current state. Safe to call from a normal task.
void ble_publish_cam_cfg(const CamCfg* cfg);
