include_directories(
	/usr/local/include/u8g2/
	./scd4x_driver/
)
set(PICO_BOARD pico_w)

cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

project(co2mon C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_C_COMPILER   /usr/bin/arm-none-eabi-gcc)
#set(CMAKE_CXX_COMPILER i586-mingw32msvc-g++)

add_subdirectory(u8g2)

add_executable(${PROJECT_NAME}
	main.c
	scd4x_driver/scd4x_i2c.c
	scd4x_driver/sensirion_common.c
	scd4x_driver/sensirion_i2c.c
	scd4x_driver/sensirion_i2c_hal.c
)

target_link_libraries(${PROJECT_NAME}
	pico_stdlib
	pico_cyw43_arch_none
	hardware_i2c
	u8g2
)
pico_add_extra_outputs(${PROJECT_NAME})
