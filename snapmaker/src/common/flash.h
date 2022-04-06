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

#ifndef SNAPMAKER_FLASH_H
#define SNAPMAKER_FLASH_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash_ex.h"

typedef struct {
  FLASH_EraseInitTypeDef erase_config;
  uint32_t  start_addr;
  uint32_t  write_addr;
  uint32_t  size;
} flash_partition_t;

extern flash_partition_t boot_data_partition;

bool flash_erase(flash_partition_t &flash_partition);
uint32_t flash_write(flash_partition_t &flash_partition, uint8_t *data, uint32_t len);

#endif
