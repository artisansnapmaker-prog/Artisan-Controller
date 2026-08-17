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
#include "stm32_def.h"

// bootloader partition
#define FLASH_BOOT_FW_START_SECTOR              (0)
#define FLASH_BOOT_FW_SECTOR_NUM                (2)
#define FLASH_BOOT_FW_ADDR                      (FLASH_BASE)
#define FLASH_BOOT_FW_SIZE                      (32 * 1024)

// bootloader data partition
#define FLASH_BOOT_DATA_START_SECTOR            (2)
#define FLASH_BOOT_DATA_SECTOR_NUM              (1)
#define FLASH_BOOT_DATA_ADDR                    (32 * 1024 + FLASH_BASE)
#define FLASH_BOOT_DATA_SIZE                    (16 * 1024)

// marlin settings partition
// #define FLASH_MARLIN_SETTINGS_START_SECTOR      (2)
// #define FLASH_MARLIN_SETTINGS_SECTOR_NUM        (1)
// #define FLASH_MARLIN_SETTINGS_ADDR              (32 * 1024 + FLASH_BASE)
// #define FLASH_MARLIN_SETTINGS_SIZE              (16 * 1024)

// app partition
// #define FLASH_APP_FW_ADDR                       (64 * 1024 + FLASH_BASE)
// #define FLASH_APP_FW_SIZE                       (592 * 1024)

// module firmware partition
#define FLASH_MODULE_FW_DOWNLOAD_ADDR           (640 * 1024 + FLASH_BASE)
#define FLASH_MODULE_FW_DOWNLOAD_SIZE           (256 * 1024)

typedef struct {
  uint32_t start;
  uint32_t size;
} flash_sector_addr_t;

// typedef struct {
//   FLASH_EraseInitTypeDef erase_config;
//   uint32_t  start_addr;
//   uint32_t  write_addr;
//   uint32_t  size;
// } flash_partition_t;

typedef struct {
  uint32_t  start_addr;
  uint32_t  write_addr;
  uint32_t  size;
} flash_partition_t;

extern flash_partition_t boot_data_partition;
extern flash_partition_t module_fw_partition;

bool flash_addr_to_sector_number(uint32_t start, uint32_t size, uint32_t &sector_start, uint32_t &sector_number);
bool flash_erase(flash_partition_t &flash_partition);
uint32_t flash_write(flash_partition_t &flash_partition, uint8_t *data, uint32_t len);

#endif
