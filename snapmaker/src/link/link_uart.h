/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_HOST_LINK_UART_H_
#define SNAPMAKER_HOST_LINK_UART_H_

#include "src/HAL/HAL.h"

class LinkUART {
  // public methods
  public:
    LinkUART() {}

    LinkUART(MSerialT *ser): serial(ser) {}

    int peek() {
      if (serial)
        return serial->peek_sec();
      else
        return -1;
    }

    int read() {
      if (serial)
        return serial->read_sec();
      else
        return -1;
    }

    size_t write(uint8_t c) {
      if (serial)
        return serial->write_sec(c);
      else
        return 0;
    }

    int available() {
      if (serial)
        return serial->available_sec();
      else
        return 0;
    }

    int set_active_channel(uint8_t new_ch) {
        if (serial)
          return serial->set_active_channel(new_ch);
        else
          return -1;
    }

    int set_sec_rx_signal(void *signal) {
        if (serial) {
          serial->set_sec_rx_signal(signal);
          return 0;
        }
        else
          return -1;
    }

    int set_sec_rx_waiting(uint16_t bytes_num) {
      if (serial) {
        serial->set_sec_rx_waiting(bytes_num);
        return 0;
      }
      else
        return -1;
    }

    int set_sec_rx_buffer(uint8_t *buffer, uint16_t size) {
      rx_buffer = buffer;
      rx_size = size;
      if (serial) {
        serial->set_sec_rx_buffer(buffer, size);
        return 0;
      }
      else
        return -1;
    }

    int set_sec_tx_buffer(uint8_t *buffer, uint16_t size) {
      tx_buffer = buffer;
      tx_size = size;
      if (serial) {
        serial->set_sec_tx_buffer(buffer, size);
        return 0;
      }
      else
        return -1;
    }

    int read_multi(uint8_t *buffer, uint16_t length) {
        if (serial)
          return serial->read_multi(MARLIN_SERIAL_CHANNEL_SECOND, buffer, length);
        else
          return -1;
    }

    int write_multi(uint8_t *buffer, uint16_t length) {
        if (serial)
          return serial->write_multi(MARLIN_SERIAL_CHANNEL_SECOND, buffer, length);
        else
          return -1;
    }

    void update_serial(MSerialT *ser) {
      if (!ser)
        return;
      serial = ser;
      serial->set_sec_rx_buffer(rx_buffer, rx_size);
      serial->set_sec_tx_buffer(tx_buffer, tx_size);
    }

    void set_serial(MSerialT *ser) {
      if (ser)
        serial = ser;
    }

    MSerialT *get_serial() {
      return  serial;
    }

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    MSerialT *serial = NULL;

    uint8_t *tx_buffer;
    uint16_t tx_size;
    uint8_t *rx_buffer;
    uint16_t rx_size;
};

extern LinkUART link_screen;
extern LinkUART link_camera;
extern LinkUART link_pc;

#endif  // #ifndef SNAPMAKER_HOST_LINK_UART_H_
