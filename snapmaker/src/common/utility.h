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
#ifndef SNAPMAKER_UTILITY_H_
#define SNAPMAKER_UTILITY_H_

#include <stdint.h>

#define time_after(a, b)                ((int)(b)-(int)(a) < 0)
#define STRCAT(s, b)                    (s##b)
#define MAX(a,b)                        ((a)>(b)?(a):(b))
#define MIN(a,b)                        ((a)<(b)?(a):(b))
#define TAB_SIZE(t, i)                  (sizeof(t) / sizeof(i))

#define LITTLE_STREAM_TO_16(buf)        ( ((buf)[1]<<8) | ((buf)[0]) )
#define LITTLE_STREAM_TO_32(buf)        ( ((buf)[3]<<24) | ((buf)[2]<<16) | ((buf)[1]<<8) | ((buf)[0]) )
#define _16_TO_LITTLE_STREAM(s, buf)    do{ ((buf)[0] = s & 0xFF); ((buf)[1] = (s>>8) & 0xFF); }while(0)
#define _32_TO_LITTLE_STREAM(s, buf)    do{ ((buf)[0] = s & 0xFF); ((buf)[1] = (s>>8) & 0xFF); ((buf)[2] = (s>>16) & 0xFF); ((buf)[3] = (s>>24) & 0xFF); }while(0)

#define LOCK(lock, wait_time)           do{ xSemaphoreTake(lock, (TickType_t)wait_time); } while(0)
#define UNLOCK(lock)                    do{ xSemaphoreGive(lock); } while(0)

#define ROUNDUP(x, y)                   ({\
                                          typeof(y) __y = y;\
                                          (((x) + (__y - 1)) / __y) * __y;		\
                                        })	

uint32_t calculate_checksum(uint8_t *buffer, uint32_t length);
uint32_t sacp_calculate_checksum(uint8_t *buffer, uint32_t length);

#endif  // #ifndef SNAPMAKER_UTILITY_H_
