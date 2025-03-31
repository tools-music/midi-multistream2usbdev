/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 rppicomidi
 * Copyright (c) 2025 Frederico Betting
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "bsp/board.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include "midi_uart_lib.h"
#include "pio_midi_uart_lib.h"
#include "midi_device_multistream.h"
//--------------------------------------------------------------------+
// This program routes 5-pin DIN MIDI IN signals A & B to USB MIDI
// virtual cables 0 & 1 on the USB MIDI Bulk IN endpoint. It also
// routes MIDI data from USB MIDI virtual cables 0-5 on the USB MIDI
// Bulk OUT endpoint to the 5-pin DIN MIDI OUT signals A-F.
// The Pico board's LED blinks in a pattern depending on the Pico's
// USB connection state (See below).
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// Number of UARTs available on the RP2040
#define HW_UART_NUM 2

#define MIDI_UART_NUM_1 0     // UART0 for MIDI
#define MIDI_UART_TX_GPIO_1 0 // GPIO pin for UART0 TX
#define MIDI_UART_RX_GPIO_1 1 // GPIO pin for UART0 RX
#define MIDI_UART_NUM_2 1     // UART1 for MIDI
#define MIDI_UART_TX_GPIO_2 4 // GPIO pin for UART1 TX
#define MIDI_UART_RX_GPIO_2 5 // GPIO pin for UART1 RX

/* Blink pattern
 * - 250 ms  : device not mountedS
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

static void led_blinking_task(void);
static void midi_pio_uart_task(void);
static void midi_hw_uart_task(void);
static void poll_midi_hw_uart_rx(bool connected);

typedef enum
{
  MIDI_A = 0,
  MIDI_B = 1,
  MIDI_C = 2,
  MIDI_D = 3,
  MIDI_E = 4,
  MIDI_F = 5,
  NUM_MIDI_PORTS
} MIDI_PORT_LIST;

// All MIDI IN and MIDI OUT ports
static void *midi_uarts[NUM_MIDI_PORTS];

// MIDI UART pin usage
static const size_t MIDI_TX_GPIO[NUM_MIDI_PORTS] = {MIDI_UART_TX_GPIO_1, MIDI_UART_TX_GPIO_2, 20, 21, 26, 27};
static const size_t MIDI_RX_GPIO[NUM_MIDI_PORTS] = {MIDI_UART_RX_GPIO_1, MIDI_UART_RX_GPIO_2, 6, 7, 8, 9};

// MIDI device address (Host)
static uint8_t midi_dev_addr = 0;

/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  tusb_init();
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  // Create the MIDI UARTs
  for (size_t n = 0; n < HW_UART_NUM; n++)
  {
    // MIDI_UART_NUM is the UART number (0 or 1 available in RP2040) to use for MIDI.
    // MIDI_A and MIDI_B are the GPIO pins used.
    midi_uarts[n] = midi_uart_configure(n, MIDI_TX_GPIO[n], MIDI_RX_GPIO[n]);
    if (midi_uarts[n] == 0)
    {
      printf("Error creating MIDI UART %zu\r\n", n);
      return 1;
    }
  }

  // Create the MIDI PIO UARTs
  for (size_t n = HW_UART_NUM; n < NUM_MIDI_PORTS - HW_UART_NUM; n++)
  {
    midi_uarts[n] = pio_midi_uart_create(MIDI_TX_GPIO[n], MIDI_RX_GPIO[n]);
    if (midi_uarts[n] == 0)
    {
      printf("Error creating UART %zu\r\n", n);
    }
  }

  printf("6-IN 6-OUT USB MIDI Patchbay\r\n");

  while (1)
  {
    tud_task(); // tinyusb device task
    tuh_task(); // tinyusb host task

    led_blinking_task();
    midi_pio_uart_task();
    midi_hw_uart_task();
  }
}

//--------------------------------------------------------------------+
// TinyUSB Device callbacks
//--------------------------------------------------------------------+
/**
 * @brief Invoked when device is mounted.
 */
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

/**
 * @brief Invoked when device is unmounted.
 */
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

/**
 * @brief  Invoked when usb bus is suspended.
 *
 * @param remote_wakeup_en if host allow us to perform remote wakeup
 *                         within 7ms, device must draw an average of
 *                         current less than 2.5 mA from bus.
 */
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

/**
 * @brief Invoked when usb bus is resumed.
 */
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
  printf("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
         dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);

  if (midi_dev_addr == 0)
  {
    // then no MIDI device is currently connected
    midi_dev_addr = dev_addr;
  }
  else
  {
    printf("A different USB MIDI Device is already connected.\r\nOnly one device at a time is supported in this program\r\nDevice is disabled\r\n");
  }
}

/**
 * @brief Invoked when device with hid interface is un-mounted.
 */
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  if (dev_addr == midi_dev_addr)
  {
    midi_dev_addr = 0;
    printf("MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
  else
  {
    printf("Unused MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
}

/**
 * @brief Invoked when data is received from the USB MIDI device.
 *
 * @note When the USB Host receives MIDI IN packets from the MIDI device,
 * this driver calls `tuh_midi_rx_cb()` to notify the application
 * that MIDI data is available. The application should read that
 * MIDI data as soon as possible. (ref: usb_midi_host documentation)
 */
void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
  if (midi_dev_addr == dev_addr)
  {
    if (num_packets != 0)
    {
      uint8_t cable_num;
      uint8_t rx[48];
      uint8_t npushed = 0;
      uint32_t nread = tuh_midi_stream_read(midi_dev_addr, &cable_num, rx, sizeof(rx));
      while (nread > 0)
      {
        if (cable_num < HW_UART_NUM)
        {
          npushed = midi_uart_write_tx_buffer(midi_uarts[cable_num], rx, nread);
        }
        else
        {
          TU_LOG1("Received a MIDI packet on cable %u", cable_num);
          npushed = 0;
          continue;
        }
        if (npushed != nread)
        {
          TU_LOG1("Warning: Dropped %lu bytes sending to MIDI In Port %c\r\n", nread - npushed, 'A' + cable_num);
        }
        nread = tuh_midi_stream_read(midi_dev_addr, &cable_num, rx, sizeof(rx));
      }
    }
  }
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
  (void)dev_addr;
}

//--------------------------------------------------------------------+
// MIDI HW task
//--------------------------------------------------------------------+
/**
 * @brief Poll the MIDI UART RX buffer and send any received bytes to the USB MIDI device.
 *
 * @param connected true if the USB MIDI device is connected.
 */
static void poll_midi_hw_uart_rx(bool connected)
{
  uint8_t rx[48];
  // MIDI Cables A / B
  // Pull any bytes received on the MIDI UART out of the receive buffer and
  // send them out via USB MIDI on virtual cable 0
  for (uint8_t cable = 0; cable < HW_UART_NUM; cable++)
  {
    uint8_t nread = midi_uart_poll_rx_buffer(midi_uarts[cable], rx, sizeof(rx));
    if (nread > 0 && connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
    {
      uint32_t nwritten = tuh_midi_stream_write(midi_dev_addr, cable, rx, nread);
      if (nwritten != nread)
      {
        TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In %c\r\n", nread - nwritten, 'A' + cable);
      }
    }
  }
}

//--------------------------------------------------------------------+
// MIDI PIO task
//--------------------------------------------------------------------+
/**
 * @brief Poll the MIDI PIO UART RX buffer and send any received bytes to the USB MIDI device.
 *
 * @param connected true if the USB MIDI device is connected.
 */
static void poll_midi_pio_uart_rx(bool connected)
{
  uint8_t rx[48];

  // MIDI Cables C / D / E / F
  for (uint8_t cable = HW_UART_NUM; cable < NUM_MIDI_PORTS - HW_UART_NUM; cable++)
  {
    uint8_t nread = pio_midi_uart_poll_rx_buffer(midi_uarts[cable], rx, sizeof(rx));
    if (nread > 0 && connected)
    {
      uint32_t nwritten = tud_midi_stream_write(cable, rx, nread);
      if (nwritten != nread)
      {
        TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In %c\r\n", nread - nwritten, 'A' + cable);
      }
    }
  }
}

/**
 * @brief Poll the USB MIDI device and send any received bytes to the MIDI PIO UART.
 *
 * @note This function is called when the USB MIDI device is connected.
 */
static void poll_pio_usb_rx(void)
{
  uint8_t rx[48];
  uint8_t cable_num;
  uint8_t npushed = 0;

  // MIDI Cables C / D / E / F
  uint32_t nread = tud_midi_demux_stream_read(&cable_num, rx, sizeof(rx));
  while (nread > 0)
  {
    if (cable_num >= HW_UART_NUM && cable_num < NUM_MIDI_PORTS)
    {
      npushed = pio_midi_uart_write_tx_buffer(midi_uarts[cable_num], rx, nread);
    }
    else
    {
      TU_LOG1("Received a MIDI packet on cable %u", cable_num);
      npushed = 0;
      continue;
    }

    if (npushed != nread)
    {
      TU_LOG1("Warning: Dropped %lu bytes sending to MIDI Out Port %c\r\n", nread - npushed, 'A' + cable_num);
    }
    nread = tud_midi_demux_stream_read(&cable_num, rx, sizeof(rx));
  }
}

//--------------------------------------------------------------------+
// MIDI UART task (main)
//--------------------------------------------------------------------+
/**
 * @brief MIDI PIO UART task to handle MIDI data transfers between the USB MIDI device and the PIO UARTs.
 */
static void midi_pio_uart_task(void)
{
  bool connected = tud_midi_mounted();
  poll_midi_pio_uart_rx(connected);

  // Device must be attached and have the endpoint ready to receive a message
  if (connected)
  {
    poll_pio_usb_rx();
  }

  // Drain PIO serial port TX buffers
  for (uint8_t cable = HW_UART_NUM; cable < NUM_MIDI_PORTS - HW_UART_NUM; cable++)
  {
    pio_midi_uart_drain_tx_buffer(midi_uarts[cable]);
  }
}

/**
 * @brief MIDI HW UART task to handle MIDI data transfers between the USB MIDI device and the HW UARTs.
 */
static void midi_hw_uart_task(void)
{
  bool connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);
  poll_midi_hw_uart_rx(connected);

  // Device must be attached and have the endpoint ready to receive a message
  if (connected)
  {
    tuh_midi_stream_flush(midi_dev_addr);
  }

  // Drain HW serial port TX buffers
  for (uint8_t cable = 0; cable < HW_UART_NUM; cable++)
  {
    midi_uart_drain_tx_buffer(midi_uarts[cable]);
  }
}

//--------------------------------------------------------------------+
// Blinking task
//--------------------------------------------------------------------+
static void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
