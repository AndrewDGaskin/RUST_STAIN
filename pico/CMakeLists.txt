# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rust_stain_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rust_stain_pico
    main.cpp
)

target_link_libraries(rust_stain_pico
    pico_stdlib
    hardware_pwm
)

pico_enable_stdio_usb(rust_stain_pico 1)
pico_enable_stdio_uart(rust_stain_pico 0)

pico_add_extra_outputs(rust_stain_pico)