cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")

include(pico_sdk_import.cmake)

project(balanceIt C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(balanceIt
  balanceIt.c
)

target_include_directories(balanceIt PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}
)

pico_enable_stdio_usb(balanceIt 1)

# Link common dependencies
target_link_libraries(balanceIt pico_stdlib hardware_timer hardware_irq hardware_i2c hardware_pwm)

# Create map/bin/hex files
pico_add_extra_outputs(balanceIt)