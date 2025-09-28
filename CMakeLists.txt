cmake_minimum_required(VERSION 3.18...3.28)

#set(PICOTOOL_FORCE_FETCH_FROM_GIT ON)
#set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../../../../pico-sdk)
set(PICO_SDK_PATH "F:\\pico\\pico-sdk")

# Set platform, board, and compiler
# ARM
set(PICO_PLATFORM rp2350)
# RISC-V
# set(PICO_PLATFORM rp2350-riscv)
# set(PICO_TOOLCHAIN_PATH ${CMAKE_CURRENT_LIST_DIR}/../../../../corev-openhw-gcc-macos-20240530)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(ScopetrexVGA C CXX ASM)

pico_sdk_init()

add_subdirectory("../../.." "DispHSTX")

add_executable(ScopetrexVGA
        src/main.cpp
)

target_link_libraries(ScopetrexVGA
        pico_stdlib
        DispHSTX
	hardware_adc
)

target_include_directories(ScopetrexVGA PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(ScopetrexVGA PRIVATE -include ${CMAKE_CURRENT_LIST_DIR}/config.h)

# RISC-V
# target_compile_definitions(ScopetrexVGA PRIVATE RISCV=1)

pico_enable_stdio_usb(ScopetrexVGA 1)
pico_enable_stdio_uart(ScopetrexVGA 0)
pico_add_extra_outputs(ScopetrexVGA)
