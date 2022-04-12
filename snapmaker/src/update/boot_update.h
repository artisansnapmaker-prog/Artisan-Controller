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
#ifndef SNAPMAKER_BOOT_UPDATE_H_
#define SNAPMAKER_BOOT_UPDATE_H_

#include <stdint.h>
#include "../common/flash.h"
#include "boot.h"

typedef void (*cmd_pf_t)(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);

typedef struct {
  uint8_t cmd_set;
  uint8_t cmd_id;
  cmd_pf_t executor;
} cmd_fun_item_t;

typedef struct {
  uint32_t offset;

  boot_info_t *boot_info;
  flash_partition_t *boot_data_partition;
  flash_partition_t *app_partition;
} update_info_t;

void update_init(boot_info_t *boot_info, 
              flash_partition_t *boot_data_partition, 
              flash_partition_t *app_partition);
void cmd_proc(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void update_loop(void);


#endif  // SNAPMAKER_BOOT_H_
