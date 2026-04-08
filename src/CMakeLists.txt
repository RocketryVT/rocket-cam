add_executable(rcam
               rocket-cam.cpp
               altimeter.cpp
               irc_tramp.cpp
               osd.cpp
)

pico_generate_pio_header(rcam ${CMAKE_CURRENT_LIST_DIR}/uart_half_duplex.pio)

target_link_libraries(rcam pico_stdlib pico_multicore pico_sync hardware_i2c hardware_pio hardware_uart pico_cyw43_arch_none)
target_include_directories(rcam PUBLIC ../include)

pico_enable_stdio_usb(rcam 1)
pico_enable_stdio_uart(rcam 0)

pico_add_extra_outputs(rcam)

