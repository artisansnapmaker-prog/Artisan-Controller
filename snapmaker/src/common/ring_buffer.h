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
//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_

#include <stdint.h>
#include <stdio.h>
#define DEFAULT_RING_BUFFER_SIZE 128


template <typename T>
class RingBuffer {
  public:
    void init(T *buffer, int32_t buf_size) {
      size = buf_size;
      head = 0;
      tail = 0;
      full = false;
      data = buffer;
    }

    int32_t insert_one(const T& element) {
      if (full) {
        return 0;
      }

      data[tail] = element;
      if (++tail >= size)
        tail = 0;

      if (tail == head) {
        full = true;
      }

      return 1;
    }

    int32_t insert_one() {
      if (full) {
        return 0;
      }

      if (++tail >= size)
        tail = 0;

      if (tail == head) {
        full = true;
      }

      return 1;
    }

    int32_t peek_one(T &val) {
      if (is_empty()) {
        return 0;
      }

      val = data[head];
      return 1;
    }

    int32_t remove_one(T &val) {
      if (is_empty()) {
        return 0;
      }

      val = data[head];
      if (++head >= size)
        head = 0;
      full = false;
      return 1;
    }

    int32_t remove_one() {
      if (is_empty()) {
        return 0;
      }

      if (++head >= size)
        head = 0;
      full = false;
      return 1;
    }

    int32_t insert_multi(T *buffer, int32_t to_insert) {
      if (is_full()) {
        return 0;
      }

      if (free() < to_insert)
        return 0;

      if (!to_insert)
        return 0;

      for (int32_t i = 0; i < to_insert; i++) {
        data[tail] = buffer[i];
        if (++tail >= size)
          tail = 0;
      }

      if (tail == head) {
        full = true;
      }
      return to_insert;
    }

    int32_t remove_multi(T *buffer, int32_t to_remove) {
      if (is_empty()) {
        return 0;
      }

      // if didn't specify number to remove, try to remove all
      if (0 == to_remove) {
        to_remove = available();
      }
      else if (available() <= to_remove) {
        to_remove = available();
      }

      for (int32_t i = 0; i < to_remove; i++) {
        buffer[i] = data[head];
        if (++head >= size) {
          head = 0;
        }
      }
      full = false;
      return to_remove;
    }

    // compiler will treat the functions who have body defined
    // in Class as inline function by default
    bool is_full() {
      return full;
    }

    bool is_empty() {
      return (head == tail) && (!full);
    }

    int32_t available() {
      int32_t delta = (int32_t)(tail - head);
      if (delta == 0 && full) {
        return size;
      }
      return (delta < 0)? (delta + size) : delta;
    }

    int32_t free() {
      int32_t delta = (int32_t)(tail - head);
      if (full) {
        return 0;
      }
      return (delta >= 0)? (size - delta) : -delta;
    }

    void reset() {
      head = tail = 0;
      full = false;
    }

  private:
    int32_t size = 0;
    int32_t head = 0;
    int32_t tail = 0;
    bool full = false;
    T *data;
};



#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
