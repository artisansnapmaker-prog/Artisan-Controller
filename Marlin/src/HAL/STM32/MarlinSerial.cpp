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

#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"
#include "MarlinSerial.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#if MB_SNAPMAKER
  #include "snapmaker.h"
#endif

#ifndef USART4
  #define USART4 UART4
#endif
#ifndef USART5
  #define USART5 UART5
#endif

#define DECLARE_SERIAL_PORT(ser_num) \
  void _rx_complete_irq_ ## ser_num (serial_t * obj); \
  MSerialT MSerial ## ser_num (true, USART ## ser_num, &_rx_complete_irq_ ## ser_num); \
  void _rx_complete_irq_ ## ser_num (serial_t * obj) { MSerial ## ser_num ._rx_complete_irq(obj); }

#if USING_HW_SERIAL1
  DECLARE_SERIAL_PORT(1)
#endif
#if USING_HW_SERIAL2
  DECLARE_SERIAL_PORT(2)
#endif
#if USING_HW_SERIAL3
  DECLARE_SERIAL_PORT(3)
#endif
#if USING_HW_SERIAL4
  DECLARE_SERIAL_PORT(4)
#endif
#if USING_HW_SERIAL5
  DECLARE_SERIAL_PORT(5)
#endif
#if USING_HW_SERIAL6
  DECLARE_SERIAL_PORT(6)
#endif
#if USING_HW_SERIAL7
  DECLARE_SERIAL_PORT(7)
#endif
#if USING_HW_SERIAL8
  DECLARE_SERIAL_PORT(8)
#endif
#if USING_HW_SERIAL9
  DECLARE_SERIAL_PORT(9)
#endif
#if USING_HW_SERIAL10
  DECLARE_SERIAL_PORT(10)
#endif
#if USING_HW_SERIALLP1
  DECLARE_SERIAL_PORT(LP1)
#endif

#include "MapleFreeRTOS1030.h"

void MarlinSerial::begin(unsigned long baud, uint8_t config) {
  HardwareSerial::begin(baud, config);
  // Replace the IRQ callback with the one we have defined
  TERN_(EMERGENCY_PARSER, _serial.rx_callback = _rx_callback);
}

// This function is Copyright (c) 2006 Nicholas Zambetti.
void MarlinSerial::_rx_complete_irq(serial_t *obj) {
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c;

  if (uart_getc(obj, &c) == 0) {

    uint16_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }

    BaseType_t if_wakeup_task = pdFALSE;
    if (active_ch == MARLIN_SERIAL_CHANNEL_SECOND) {
      if (sec_rx_signal) {
        if (available_sec() >= sec_rx_waiting) {
          sec_rx_waiting = 0xFFFF;
          xSemaphoreGiveFromISR((SemaphoreHandle_t)sec_rx_signal, &if_wakeup_task);
          portYIELD_FROM_ISR(if_wakeup_task);
        }
      }
      return;
    }

    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser.update(static_cast<MSerialT*>(this)->emergency_state, c);
    #endif
  }
}


int MarlinSerial::_sec_tx_complete_irq(serial_t *obj) {
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte
  obj->tx_tail = (obj->tx_tail + 1) % SACP_PDU_MAX_SIZE;

  if (obj->tx_head == obj->tx_tail) {
    return -1;
  }

  return 0;
}

// implemented APIs for marlin
int MarlinSerial::peek(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    return HardwareSerial::peek();
  }
  else
    return -1;
}

int MarlinSerial::read(void) {
  // original hardware channel
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
      return HardwareSerial::read();
  }
  else
    return -1;
}

void MarlinSerial::flush(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    HardwareSerial::flush();
  }
}

size_t MarlinSerial::write(uint8_t c) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    return HardwareSerial::write(c);
  }
  else {
    uint16_t i = (orig_tx_head + 1) % SERIAL_TX_BUFFER_SIZE;

    // If the output buffer is full, there's nothing for it other than to
    // wait for the interrupt handler to empty it a bit
    if (i == sec_tx_tail) {
      // nop, the interrupt handler will free up space for us
      return -1;
    }

    _tx_buffer[orig_tx_head] = c;
    orig_tx_head = i;

    return 1;
  }
}

int MarlinSerial::available(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    return HardwareSerial::available();
  }
  else {
    // if current channel is not origianl, return 0
    return 0;
  }
}


// API for second channel
int MarlinSerial::peek_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return -1;

  return HardwareSerial::peek();
}

int MarlinSerial::read_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return -1;

  enableHalfDuplexRx();
  // if the head isn't ahead of the tail, we don't have any characters
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  }
  else {
    unsigned char c = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (uint16_t)(_serial.rx_tail + 1) % sec_rx_size;
    return c;
  }
}

void MarlinSerial::flush_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return;

  HardwareSerial::flush();
}

size_t MarlinSerial::write_sec(uint8_t c) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return 0;


  _written = true;

  uint16_t i = (_serial.tx_head + 1) % sec_tx_size;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  while (i == _serial.tx_tail) {
    // nop, the interrupt handler will free up space for us
  }

  _serial.tx_buff[_serial.tx_head] = c;
  _serial.tx_head = i;

  if (!serial_tx_active(&_serial)) {
    uart_attach_tx_callback(&_serial, _sec_tx_complete_irq);
  }

  return 1;
}

int MarlinSerial::available_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return 0;

  return ((unsigned int)(sec_rx_size + _serial.rx_head - _serial.rx_tail)) % sec_rx_size;
}

void MarlinSerial::set_sec_rx_waiting(uint16_t waiting_bytes) {
  taskENTER_CRITICAL();
  sec_rx_waiting = waiting_bytes;
  taskEXIT_CRITICAL();
}

int MarlinSerial::read_multi(uint8_t ch, uint8_t *buffer, uint16_t length) {
  if (ch != active_ch)
    return 0;

  volatile uint16_t size = 0;

  if (ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    size = SERIAL_RX_BUFFER_SIZE;
      if (available() == 0)
        return 0;

      if (available() < length)
        length = available();
  }
  else {
    size = sec_rx_size;

    if (available_sec() == 0)
      return 0;

    if (available_sec() < length)
      length = available_sec();
  }

  for (int i = 0; i < length; i++) {
    buffer[i] = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (uint16_t)(_serial.rx_tail + 1) % size;
  }

  return length;
}


int MarlinSerial::write_multi(uint8_t ch, uint8_t *buffer, uint16_t length) {
  if (ch != active_ch)
    return -1;

  volatile uint16_t buffer_size = 0;
  volatile uint16_t free_size = 0;

  uint16_t head = _serial.tx_head;
  uint16_t tail = _serial.tx_tail;

  if (ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    buffer_size = SERIAL_TX_BUFFER_SIZE;
  }
  else {
    buffer_size = sec_tx_size;
  }

  if (head >= tail) {
    free_size =  buffer_size - 1 - head + tail;
  }
  else {
    free_size =  tail - head - 1;
  }

  if (free_size == 0)
    return free_size;

  if (free_size < length)
    length = free_size;

  for (int i = 0; i < length; i++) {
    head = (_serial.tx_head + 1) % buffer_size;

    while (head == _serial.tx_tail) {
      // nop, the interrupt handler will free up space for us
    }
    _serial.tx_buff[head] = buffer[i];
    _serial.tx_head = head;
  }

  if (!serial_tx_active(&_serial)) {
    if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL)
      uart_attach_tx_callback(&_serial, _tx_complete_irq);
    else
      uart_attach_tx_callback(&_serial, _sec_tx_complete_irq);
  }

  return length;
}


int MarlinSerial::set_active_channel(uint8_t new_ch) {

  if (new_ch == active_ch)
    return 0;

  if (new_ch >= MARLIN_SERIAL_CHANNEL_INVALID)
    return -1;

  // IRQ level of UART is 10, could be disable by below API
  taskENTER_CRITICAL();

  if (new_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    // save index firstly
    sec_rx_head = _serial.rx_head;
    sec_rx_tail = _serial.rx_tail;
    sec_tx_head = _serial.tx_head;
    sec_tx_tail = _serial.tx_tail;
    // clear signal
    if (sec_rx_signal)
      while (xSemaphoreTake((SemaphoreHandle_t)sec_rx_signal, 0) != pdFALSE);

    // clear orignal buffer index
    orig_rx_head = 0;
    orig_rx_tail = 0;
    orig_tx_head = 0;
    orig_tx_tail = 0;

    // update current buffer to original
    _serial.rx_buff = _rx_buffer;
    _serial.rx_head = orig_rx_head;
    _serial.rx_tail = orig_rx_tail;
    _serial.tx_buff = _tx_buffer;
    _serial.tx_head = orig_tx_head;
    _serial.tx_tail = orig_tx_tail;
  }
  else {
    // save index firstly
    orig_rx_head = _serial.rx_head;
    orig_rx_tail = _serial.rx_tail;
    orig_tx_head = _serial.tx_head;
    orig_tx_tail = _serial.tx_tail;

    // clear orignal buffer index
    sec_rx_head = 0;
    sec_rx_tail = 0;
    sec_tx_head = 0;
    sec_tx_tail = 0;

    // buffer for second channel
    _serial.rx_buff = sec_rx_buffer;
    _serial.rx_head = sec_rx_head;
    _serial.rx_tail = sec_rx_tail;
    _serial.tx_buff = sec_tx_buffer;
    _serial.tx_head = sec_tx_head;
    _serial.tx_tail = sec_tx_tail;
  }

  active_ch = new_ch;

  taskEXIT_CRITICAL();

  return 0;
}

#endif // HAL_STM32
