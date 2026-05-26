cmake_minimum_required(VERSION 3.22)
project(blinky_rt1186)

find_package(RT1186CMake REQUIRED)

set(TARGET_NAME blinky_cm33)

add_executable(${TARGET_NAME}
    src/led_blinky.cpp
    src/hardware_init.cpp
)

target_link_libraries(${TARGET_NAME} PRIVATE RT1186CMake::driverlib)

rt1186_finalize(${TARGET_NAME})


add_custom_target(flash
    COMMAND LinkServer flash MIMXRT1186:FRDM-IMXRT1186 load blinky_cm33.elf
    DEPENDS ${TARGET_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)