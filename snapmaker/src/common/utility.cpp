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
#include "utility.h"


uint32_t calculate_checksum(uint8_t *buffer, uint32_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (uint32_t j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return checksum;
}

void snap_memcpy(void *dst, void *src, uint32_t len) {
  uint8_t *d, *s;

  d = (uint8_t *)dst;
  s = (uint8_t *)src;
  for (uint32_t i = 0; i < len; i++) {
    d[i] = s[i];
  }
}