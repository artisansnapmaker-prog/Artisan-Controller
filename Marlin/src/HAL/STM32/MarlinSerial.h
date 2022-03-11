/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#include "../../core/serial_hook.h"

enum MarlinSeralChannel {
  MARLIN_SERIAL_CHANNEL_ORIGINAL,
  MARLIN_SERIAL_CHANNEL_SECOND,

  MARLIN_SERIAL_CHANNEL_INVALID
};

typedef void (*usart_rx_callback_t)(serial_t * obj);

struct MarlinSerial : public HardwareSerial {
  MarlinSerial(void *peripheral, usart_rx_callback_t rx_callback) :
      HardwareSerial(peripheral), _rx_callback(rx_callback)
  { }

  void begin(unsigned long baud, uint8_t config);
  inline void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }

  void _rx_complete_irq(serial_t *obj);

  static int _sec_tx_complete_irq(serial_t *obj);

  int peek(void);
  int read(void);
  void flush(void);
  size_t write(uint8_t);
  int available(void);

  int peek_sec(void);
  int read_sec(void);
  void flush_sec(void);
  size_t write_sec(uint8_t);
  int available_sec(void);

  int set_active_channel(uint8_t new_ch);
  uint8_t get_active_channel() {
    return active_ch;
  }

  void set_sec_rx_signal(void *signal) {
    if (!signal)
      sec_rx_signal = signal;
  }

  void set_sec_rx_buffer(uint8_t *buffer, uint16_t size) {
    if (sec_rx_buffer)
      return;

    sec_rx_buffer = buffer;
    sec_rx_size = size;
  }

  void set_sec_tx_buffer(uint8_t *buffer, uint16_t size) {
    if (sec_tx_buffer)
      return;

    sec_tx_buffer = buffer;
    sec_tx_size = size;
  }

  void set_sec_rx_waiting(uint16_t waiting_bytes);

  int read_multi(uint8_t ch, uint8_t *buffer, uint16_t length);
  int write_multi(uint8_t ch, uint8_t *buffer, uint16_t length);

protected:
  usart_rx_callback_t _rx_callback;

  uint8_t active_ch = MARLIN_SERIAL_CHANNEL_ORIGINAL;

  uint16_t orig_rx_head = 0;
  uint16_t orig_rx_tail = 0;

  uint16_t orig_tx_head = 0;
  uint16_t orig_tx_tail = 0;

  uint8_t *sec_rx_buffer = NULL;
  uint16_t sec_rx_size = 0;
  uint16_t sec_rx_head = 0;
  uint16_t sec_rx_tail = 0;

  void *sec_rx_signal = NULL;
  uint16_t sec_rx_waiting = 0;

  uint8_t *sec_tx_buffer = NULL;
  uint16_t sec_tx_size = 0;
  uint16_t sec_tx_head = 0;
  uint16_t sec_tx_tail = 0;

  void *read_lock = NULL;
  void *write_lock = NULL;
};

typedef Serial1Class<MarlinSerial> MSerialT;
extern MSerialT MSerial1;
extern MSerialT MSerial2;
extern MSerialT MSerial3;
extern MSerialT MSerial4;
extern MSerialT MSerial5;
extern MSerialT MSerial6;
extern MSerialT MSerial7;
extern MSerialT MSerial8;
extern MSerialT MSerial9;
extern MSerialT MSerial10;
extern MSerialT MSerialLP1;
