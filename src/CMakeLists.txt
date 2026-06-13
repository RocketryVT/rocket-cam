add_executable(rcam
               rocket-cam.cpp
               altimeter.cpp
               irc_tramp.cpp
               osd.cpp
               ble.cpp
               # SIGMA2 protobuf (nanopb), generated + checked in
               ${LIBS_ROOT}/packets/SIGMA2/SIGMA2_Proto.pb.c
               ${NANOPB_PATH}/pb_common.c
               ${NANOPB_PATH}/pb_encode.c
               ${NANOPB_PATH}/pb_decode.c
)

pico_generate_pio_header(rcam ${CMAKE_CURRENT_LIST_DIR}/uart_half_duplex.pio)

# Compile the BLE GATT database (sigma2_cam.gatt -> sigma2_cam.h)
pico_btstack_make_gatt_header(rcam PRIVATE ${CMAKE_CURRENT_LIST_DIR}/sigma2_cam.gatt)

target_compile_definitions(rcam PRIVATE
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
)

target_include_directories(rcam PUBLIC
    ../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # FreeRTOSConfig.h, btstack_config.h
    ${LIBS_ROOT}/packets/SIGMA2              # SIGMA2_Proto.pb.h
    ${NANOPB_PATH}                           # pb.h, pb_encode.h, pb_decode.h
)

target_link_libraries(rcam
    pico_stdlib
    pico_sync
    hardware_i2c
    hardware_pio
    hardware_uart
    pico_cyw43_arch_none                      # CYW43_LWIP=0 + threadsafe background async context
    pico_btstack_ble                          # LE GATT server
    pico_btstack_cyw43                        # BTstack <-> cyw43 transport (auto-inits run loop in cyw43_arch_init)
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(rcam 1)
pico_enable_stdio_uart(rcam 0)

pico_add_extra_outputs(rcam)
