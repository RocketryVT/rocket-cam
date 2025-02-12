add_executable(rcam
               rocket-cam.cpp
               altimeter.cpp
)

target_link_libraries(rcam pico_stdlib pico_multicore pico_sync hardware_i2c pico_cyw43_arch_none)
target_include_directories(rcam PUBLIC ../include)

pico_enable_stdio_usb(rcam 0)
pico_enable_stdio_uart(rcam 0)

pico_add_extra_outputs(rcam)

