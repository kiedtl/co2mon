cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(rf24/CMakeLists.txt)
include(rf24network/CMakeLists.txt)
include(rf24mesh/CMakeLists.txt)
pico_sdk_init()
include_directories(
	/usr/local/include/u8g2/
	./scd4x_driver/
)

project(co2mon C CXX ASM)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(MESH_NOMASTER)
add_compile_definitions(PICO_W)

add_subdirectory(u8g2)

add_executable(${PROJECT_NAME}
	main.c
	wrapper.cpp
	scd4x_driver/scd4x_i2c.c
	scd4x_driver/sensirion_common.c
	scd4x_driver/sensirion_i2c.c
	scd4x_driver/sensirion_i2c_hal.c
)

target_link_libraries(${PROJECT_NAME}
	pico_stdlib
	pico_cyw43_arch_none
	hardware_i2c
	hardware_spi
	RF24
	RF24Network
	RF24Mesh
	u8g2
)
pico_add_extra_outputs(${PROJECT_NAME})
