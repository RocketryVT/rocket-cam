#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Lean BTstack configuration for rocket-cam: BLE peripheral / GATT server only.
// No Classic, no audio. Modeled on the pico-sdk picow_ble examples.

// ---- BTstack features --------------------------------------------------------
#define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// ---- Buffer sizes ------------------------------------------------------------
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (255 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

#define MAX_NR_GATT_CLIENTS 0
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_SERVICES  2
#define MAX_NR_L2CAP_CHANNELS  2
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1

// Limit number of ACL/SCO buffers used by the stack to avoid cyw43 bus overrun.
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// HCI Controller to Host flow control to avoid cyw43 shared-bus overrun.
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// Non-volatile storage for the LE device DB (TLV over flash).
#define NVM_NUM_DEVICE_DB_ENTRIES 4
#define NVM_NUM_LINK_KEYS 4

// We do not give BTstack a malloc, so use a fixed-size ATT DB.
#define MAX_ATT_DB_SIZE 512

// ---- HAL / platform ----------------------------------------------------------
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS 1000

#define ENABLE_SOFTWARE_AES128
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS

#endif // BTSTACK_CONFIG_H
