cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(motor_control)

pico_generate_pio_header(motor_control ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_sources(motor_control PRIVATE motor_control.cpp)

target_link_libraries(motor_control PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        )

pico_enable_stdio_usb(motor_control 1)

pico_add_extra_outputs(motor_control)
