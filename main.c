/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 rppicomidi
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
#include "pio_midi_uart_lib.h"
#include "midi_uart_lib.h"
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
// UART selection Pin mapping. You can move these for your design if you want to
// Make sure all these values are consistent with your choice of midi_uart
// The default is to use UART 1, but you are free to use UART 0 if you make
// the changes in the CMakeLists.txt file or in your environment. Note
// that if you use UART0, then serial port debug will not be enabled
#ifndef MIDI_UART_NUM
#define MIDI_UART_NUM 1
#endif
#ifndef MIDI_UART_TX_GPIO
#define MIDI_UART_TX_GPIO 4
#endif
#ifndef MIDI_UART_RX_GPIO
#define MIDI_UART_RX_GPIO 5
#endif

// Number of PIO UARTs available
#define PIO_UART_NUM 4

// Number of HW UARTs available
#define HW_UART_NUM 1

/* Blink pattern
 * - 250 ms  : device not mounted
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
static void midi_task(void);


// MIDI UART PORTS
static void *pio_midi_uarts[PIO_UART_NUM];
static void *hw_midi_uarts[HW_UART_NUM];

// MIDI UART pin usage
static const uint MIDI_OUT_A_GPIO = 18;
static const uint MIDI_IN_A_GPIO = 6;
static const uint MIDI_OUT_B_GPIO = 19;
static const uint MIDI_IN_B_GPIO = 7;
static const uint MIDI_OUT_C_GPIO = 20;
static const uint MIDI_IN_C_GPIO = 8;
static const uint MIDI_OUT_D_GPIO = 21;
static const uint MIDI_IN_D_GPIO = 9;

// MIDI HW UART pin usage
static const uint MIDI_OUT_E_GPIO = MIDI_UART_TX_GPIO;
static const uint MIDI_IN_E_GPIO = MIDI_UART_RX_GPIO;
/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  // Create the MIDI PIO UARTs instances
  pio_midi_uarts[0] = pio_midi_uart_create(MIDI_OUT_A_GPIO, MIDI_IN_A_GPIO);
  pio_midi_uarts[1] = pio_midi_uart_create(MIDI_OUT_B_GPIO, MIDI_IN_B_GPIO);
  pio_midi_uarts[2] = pio_midi_uart_create(MIDI_OUT_C_GPIO, MIDI_IN_C_GPIO);
  pio_midi_uarts[3] = pio_midi_uart_create(MIDI_OUT_D_GPIO, MIDI_IN_D_GPIO);

  // Create the MIDI HW UARTs instances
  hw_midi_uarts[0] = midi_uart_configure(MIDI_UART_NUM, MIDI_OUT_E_GPIO, MIDI_IN_E_GPIO);

  printf("6-IN 6-OUT USB MIDI Device adapter\r\n");
  //
  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
    midi_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+
static void poll_midi_uarts_rx(bool connected)
{
  uint8_t rx[48];
  // Pull any bytes received on the MIDI UART out of the receive buffer and
  // send them out via USB MIDI on virtual cable 0
  for (uint8_t cable = 0; cable < PIO_UART_NUM; cable++)
  {
    uint8_t nread = pio_midi_uart_poll_rx_buffer(pio_midi_uarts[cable], rx, sizeof(rx));
    if (nread > 0 && connected)
    {
      uint32_t nwritten = tud_midi_stream_write(cable, rx, nread);
      if (nwritten != nread)
      {
        TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In %c\r\n", nread - nwritten, 'A' + cable);
      }
    }
  }

  for (uint8_t cable = 0; cable < HW_UART_NUM; cable++)
  {
    uint8_t nread = midi_uart_poll_rx_buffer(hw_midi_uarts[cable], rx, sizeof(rx));
    if (nread > 0 && connected)
    {
      uint32_t nwritten = tud_midi_stream_write(cable + PIO_UART_NUM, rx, nread);
      if (nwritten != nread)
      {
        TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In %c\r\n", nread - nwritten, 'A' + cable + PIO_UART_NUM);
      }
    }
  }
}

static void poll_usb_rx(bool connected)
{
  // device must be attached and have the endpoint ready to receive a message
  if (!connected)
  {
    return;
  }
  uint8_t rx[48];
  uint8_t cable_num;
  uint8_t npushed = 0;
  uint32_t nread = tud_midi_demux_stream_read(&cable_num, rx, sizeof(rx));
  while (nread > 0)
  {
    if (cable_num < PIO_UART_NUM)
    {
      // then it is MIDI OUT A / B / C / D
      npushed = pio_midi_uart_write_tx_buffer(pio_midi_uarts[cable_num], rx, nread);
    }
    else if (cable_num < PIO_UART_NUM + HW_UART_NUM)
    {
      // then it is MIDI OUT E
      npushed = midi_uart_write_tx_buffer(hw_midi_uarts[cable_num - PIO_UART_NUM], rx, nread);
    }
    else
    {
      // then it is an invalid cable number
      TU_LOG1("Invalid cable number %u\r\n", cable_num);
      nread = tud_midi_demux_stream_read(&cable_num, rx, sizeof(rx));
      continue;
    }
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

static void drain_serial_port_tx_buffers()
{
  uint8_t cable;
  for (cable = 0; cable < PIO_UART_NUM; cable++)
  {
    pio_midi_uart_drain_tx_buffer(pio_midi_uarts[cable]);
  }
  for (cable = 0; cable < HW_UART_NUM; cable++)
  {
    midi_uart_drain_tx_buffer(hw_midi_uarts[cable]);
  }
}

static void midi_task(void)
{
  bool connected = tud_midi_mounted();
  poll_midi_uarts_rx(connected);
  poll_usb_rx(connected);
  drain_serial_port_tx_buffers();
}

//--------------------------------------------------------------------+
// BLINKING TASK
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
