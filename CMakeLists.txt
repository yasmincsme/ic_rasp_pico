# Generated CMake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize pico_sdk from installed location
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(ic_rasp_pico C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Add the main executable
add_executable(ic_rasp_pico 
    ic_rasp_pico.cpp
)

# Set name and version
pico_set_program_name(ic_rasp_pico "ic_rasp_pico")
pico_set_program_version(ic_rasp_pico "0.1")

# Enable USB/UART communication
pico_enable_stdio_uart(ic_rasp_pico 1)
pico_enable_stdio_usb(ic_rasp_pico 1)

# Include current directory
target_include_directories(ic_rasp_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link with necessary libraries
target_link_libraries(ic_rasp_pico
    pico_stdlib
    hardware_i2c
    hardware_timer
    hardware_adc
)

# Generate UF2 and other output formats
pico_add_extra_outputs(ic_rasp_pico)
