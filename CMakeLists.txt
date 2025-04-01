# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

set(BOARD pico_sdk)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(midi-multstream2usbdev C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(PROJECT midi-multstream2usbdev)

project(${PROJECT})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pio_midi_uart_lib)  # MIDI PIO UART library
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/midi_uart_lib)  # MIDI HW UART library

add_executable(${PROJECT}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
  ${CMAKE_CURRENT_LIST_DIR}/midi_device_multistream.c
)

# Add the HW MIDI UART config
if(DEFINED ENV{MIDI_UART_NUM})
    set(MIDI_UART_NUM $ENV{MIDI_UART_NUM})
endif()
if(DEFINED ENV{MIDI_UART_TX_GPIO})
    set(MIDI_UART_TX_GPIO $ENV{MIDI_UART_TX_GPIO})
endif()
if(DEFINED ENV{MIDI_UART_RX_GPIO})
    set(MIDI_UART_RX_GPIO $ENV{MIDI_UART_RX_GPIO})
endif()
if (DEFINED MIDI_UART_NUM)
    if (${MIDI_UART_NUM} EQUAL 1)
        # MIDI is UART 1, so can enable UART 0 debugging
        pico_enable_stdio_uart(${PROJECT} 1)
        if(DEFINED MIDI_UART_TX_GPIO AND DEFINED MIDI_UART_RX_GPIO)
            target_compile_options(${PROJECT} PRIVATE -DMIDI_UART_NUM=${MIDI_UART_NUM} -DMIDI_UART_TX_GPIO=${MIDI_UART_TX_GPIO} -DMIDI_UART_RX_GPIO=${MIDI_UART_RX_GPIO})
        endif()
    elseif(NOT ${MIDI_UART_NUM} EQUAL 0)
        message(FATAL_ERROR "Legal values for MIDI_UART_NUM are 0 or 1, CMake will exit." )
    elseif(NOT DEFINED MIDI_UART_TX_GPIO OR NOT DEFINED MIDI_UART_RX_GPIO)
        message(FATAL_ERROR "You must define MIDI_UART_TX_GPIO and MIDI_UART_RX_GPIO if you define MIDI_UART_NUM 0, CMake will exit." )
    else()
        message("UART debugging is disabled if you use UART 0 for MIDI")
        pico_enable_stdio_uart(${PROJECT} 0)
        target_compile_options(${PROJECT} PRIVATE -DMIDI_UART_NUM=${MIDI_UART_NUM} -DMIDI_UART_TX_GPIO=${MIDI_UART_TX_GPIO} -DMIDI_UART_RX_GPIO=${MIDI_UART_RX_GPIO})
    endif()
else()
    # not defined, so will be UART 1. Can enable UART 0 debugging
    pico_enable_stdio_uart(${PROJECT} 1)
endif()


target_include_directories(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/preprocessor/include
)

target_link_options(${PROJECT} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${PROJECT} PRIVATE -Wall -Wextra -DCFG_TUSB_DEBUG=1)
target_link_libraries(${PROJECT} pio_midi_uart_lib midi_uart_lib tinyusb_device tinyusb_board pico_stdlib)

pico_add_extra_outputs(${PROJECT})
