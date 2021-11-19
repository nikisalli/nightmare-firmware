cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")

include(pico-sdk/pico_sdk_init.cmake)

project(nv)

pico_sdk_init()

FILE(GLOB c_files src/*.c)
add_executable(nv ${c_files})

target_include_directories(nv PUBLIC ./src pico-sdk/lib/tinyusb/src)

target_link_libraries(nv
	pico_multicore
	pico_stdlib
	tinyusb_device
	hardware_adc
	hardware_spi
	hardware_sync)

pico_add_extra_outputs(nv)