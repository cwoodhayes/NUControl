cmake_minimum_required(VERSION 3.22)
project(blinky_rt1186)

find_package(RT1186CMake REQUIRED)

set(TARGET_NAME blinky_cm33)

add_executable(${TARGET_NAME}
    src/led_blinky.cpp
    src/hardware_init.cpp
    src/current_sense_rt1186.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core
)

target_link_libraries(${TARGET_NAME} PRIVATE RT1186CMake::driverlib)

target_link_options(${TARGET_NAME} PRIVATE -Wl,--defsym=__stack_size__=0x2000)

rt1186_finalize(${TARGET_NAME})


# convenience commands
add_custom_target(flash
    COMMAND LinkServer flash MIMXRT1186:FRDM-IMXRT1186 load blinky_cm33.elf
    DEPENDS ${TARGET_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(debug
    COMMAND LinkServer gdbserver MIMXRT1186:FRDM-IMXRT1186 --core=cm33 --gdb-port 3333
    DEPENDS ${TARGET_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
# then run: arm-none-eabi-gdb platforms/rt1186/build/blinky_cm33.elf
add_custom_target(erase
    COMMAND LinkServer flash MIMXRT1186:FRDM-IMXRT1186 erase
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)