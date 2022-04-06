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
#ifndef SNAPMAKER_BOOT_H_
#define SNAPMAKER_BOOT_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../common/flash.h"

#define BOOT_MODE_FACTORY_BURNING     (0xAA01)
#define BOOT_MODE_APP                 (0xAA02)
#define BOOT_MODE_COPY                (0xAA03)                
#define BOOT_MODE_UPDATING            (0xAA04)
#define BOOT_DELAY_SECODE             (6)

typedef struct __attribute__ ((packed)) {
  uint8_t magic_str[21];
  uint8_t ver;
  uint8_t type;
  uint16_t start_index;
  uint16_t end_index;
  uint8_t fw_ver_str[32];
  uint8_t timestamp_str[20];
  uint16_t boot_mode;
  uint32_t fw_len;
  uint32_t fw_checksum;
  uint32_t fw_runaddr;
  uint32_t boot_data_checksum; 
} boot_info_t;

typedef void (*pf)(void);


bool load_boot_info(void);
void setup(void);
void boot_app(void);
void copy_to_run_slot(void);
void trans_fw_loop(void);
void loop(void);


#endif  // SNAPMAKER_BOOT_H_
