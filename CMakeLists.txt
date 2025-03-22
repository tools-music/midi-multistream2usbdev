cmake_minimum_required(VERSION 3.13)

set(BOARD pico_sdk)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(midi2usbdev C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(PROJECT midi-multstream2usbdev)

project(${PROJECT})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/usb_midi_host)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pio_midi_uart_lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/midi_uart_lib)

add_executable(${PROJECT}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
  ${CMAKE_CURRENT_LIST_DIR}/midi_device_multistream.c
)

if(DEFINED ENV{MIDI_UART_NUM})
    set(MIDI_UART_NUM $ENV{MIDI_UART_NUM})
endif()
if(DEFINED ENV{MIDI_UART_TX_GPIO})
    set(MIDI_UART_TX_GPIO $ENV{MIDI_UART_TX_GPIO})
endif()
if(DEFINED ENV{MIDI_UART_RX_GPIO})
    set(MIDI_UART_RX_GPIO $ENV{MIDI_UART_RX_GPIO})
endif()

target_include_directories(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/preprocessor/include
)

target_link_options(${PROJECT} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${PROJECT} PRIVATE -Wall -Wextra -DCFG_TUSB_DEBUG=1)
target_link_libraries(${PROJECT} 
  pio_midi_uart_lib
  midi_uart_lib
  usb_midi_host
  tinyusb_device
  tinyusb_board
  tinyusb_host
  pico_stdlib
  ring_buffer_lib
  usb_midi_host_app_driver
)

pico_enable_stdio_uart(${PROJECT} 0)
target_compile_options(${PROJECT} PRIVATE
  -DMIDI_UART_NUM_1=0  # UART0 for MIDI
  -DMIDI_UART_TX_GPIO_1=0  # GPIO pin for UART0 TX
  -DMIDI_UART_RX_GPIO_1=1  # GPIO pin for UART0 RX
  -DMIDI_UART_NUM_2=1  # UART1 for MIDI
  -DMIDI_UART_TX_GPIO_2=4  # GPIO pin for UART1 TX
  -DMIDI_UART_RX_GPIO_2=5  # GPIO pin for UART1 RX
)

pico_add_extra_outputs(${PROJECT})
