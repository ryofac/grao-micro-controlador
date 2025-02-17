# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(conexao_servidor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(conexao_servidor conexao_servidor.c 
${CMAKE_CURRENT_LIST_DIR}/lib/pico-ssd1306/ssd1306.c
${CMAKE_CURRENT_LIST_DIR}/lib/doglab/doglab.c
)

pico_set_program_name(conexao_servidor "conexao_servidor")
pico_set_program_version(conexao_servidor "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(conexao_servidor 0)
pico_enable_stdio_usb(conexao_servidor 1)

# Add the standard library to the build
target_link_libraries(conexao_servidor
        pico_stdlib
        hardware_adc
)

# Add the standard include files to the build
target_include_directories(conexao_servidor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lib
        ${CMAKE_CURRENT_LIST_DIR}/lib/pico-ssd1306 
        ${CMAKE_CURRENT_LIST_DIR}/lib/doglab 
)

# Add any user requested libraries
target_link_libraries(conexao_servidor 
        hardware_i2c
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(conexao_servidor)

