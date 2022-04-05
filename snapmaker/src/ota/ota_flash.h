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

#ifndef OTA_FLASH_H
#define OTA_FLASH_H


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


#define BOOT_INFO_ADDR                (16 * 1024 + FLASH_BASE)
#define BOOT_INFO_SIZE                (16 * 1024)
#define DOWNLOAD_SLOT_ADDR            (48 * 1024 + FLASH_BASE)
#define DOWNLOAD_SLOT_SZIE            (464 * 1024)
#define FLASH_TAB_SIZE                (12)

struct flash_item {
  uint32_t  start_addr;
  uint32_t  end_addr;
};

bool flash_erase_boot_data(void);
bool flash_word_write(uint32_t addr, uint32_t *data, uint32_t word_len);
void flash_word_read(uint32_t addr, uint32_t *data, uint32_t word_len);

#endif
