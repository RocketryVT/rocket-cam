add_executable(rcam
               rocket-cam.cpp
               altimeter.cpp
               irc_tramp.cpp
               osd.cpp
)

pico_generate_pio_header(rcam ${CMAKE_CURRENT_LIST_DIR}/uart_half_duplex.pio)

target_compile_definitions(rcam PRIVATE
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
)

target_include_directories(rcam PUBLIC
    ../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # FreeRTOSConfig.h
)

target_link_libraries(rcam
    pico_stdlib
    pico_sync
    hardware_i2c
    hardware_pio
    hardware_uart
    pico_cyw43_arch_none
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(rcam 1)
pico_enable_stdio_uart(rcam 0)

pico_add_extra_outputs(rcam)
