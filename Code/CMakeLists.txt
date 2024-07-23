cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(rpi_pico_drone)

pico_sdk_init()

add_executable(rpi_pico_drone
    main.c
)

target_link_libraries(rpi_pico_drone pico_stdlib hardware_pwm hardware_spi hardware_adc)

pico_enable_stdio_usb(rpi_pico_drone 1)
pico_enable_stdio_uart(rpi_pico_drone 0)

pico_add_extra_outputs(rpi_pico_drone)