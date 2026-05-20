cmake_minimum_required(VERSION 3.22)
project(blinky_rt1186)

find_package(RT1186CMake REQUIRED)

set(TARGET_NAME blinky_cm33)

add_executable(${TARGET_NAME}
    src/led_blinky.c
    src/hardware_init.c
)

target_link_libraries(${TARGET_NAME} PRIVATE RT1186CMake::driverlib)

rt1186_finalize(${TARGET_NAME})
