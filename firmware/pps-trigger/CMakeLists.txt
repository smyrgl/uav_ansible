cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(pps_trigger C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(pps_trigger main.c)
pico_generate_pio_header(pps_trigger ${CMAKE_CURRENT_LIST_DIR}/trigger.pio)
pico_generate_pio_header(pps_trigger ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
target_link_libraries(pps_trigger pico_stdlib pico_rand hardware_pio hardware_timer hardware_clocks hardware_watchdog)
# Reports go over USB CDC. A host that is not reading must never hold the loop:
# writes give up after 2 ms (default 500 ms) and are skipped while no host is connected.
target_compile_definitions(pps_trigger PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000)
pico_enable_stdio_usb(pps_trigger 1)
pico_enable_stdio_uart(pps_trigger 0)
pico_set_program_name(pps_trigger "uav-pps-trigger")
pico_set_program_description(pps_trigger "PPS-disciplined 60 Hz trigger train for the Hadron 640R+ (one-shot slave sync), UTC-labelled over USB CDC")
pico_add_extra_outputs(pps_trigger)
