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

/*****
  ** @file     : ota_flash.c/h
  ** @brief    : Flash存储库
  ** @versions : newest
  ** @time     : newest
  ** @reviser  : unli (WuHu China)
  ** @explain  : null
*****/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "flash.h"


/*************************************************************** Flash_Start ***************************************************************/
bool flash_erase_boot_data(flash_partition_t &flash_partition) {
  bool ret = true;
  uint32_t page_error;
  
  HAL_FLASH_Unlock();
  if (HAL_OK != HAL_FLASHEx_Erase(&(flash_partition.erase_config), &page_error)) {
    ret = false;
  }
  if (0xFFFFFFFF != page_error) {
    ret = false;
  }
  FLASH_WaitForLastOperation(HAL_MAX_DELAY);
  HAL_FLASH_Lock();
  return ret;
}

bool flash_word_write(flash_partition_t &flash_partition, uint8_t *data, uint32_t len) {
  uint32_t flash_free_size;

  HAL_FLASH_Unlock();
  while(len) {
    if (flash_partition.write_addr < flash_partition.start_addr) {
      flash_free_size = flash_partition.size;
      flash_partition.write_addr = flash_partition.start_addr;
    }
    else {
      flash_free_size = flash_partition.size - (flash_partition.write_addr - flash_partition.start_addr;
    }
    
    if (0 == flash_free_size) {
      break;
    }
    if (flash_partition.size - (flash_partition.write_addr - flash_partition.start_addr)) {

    }
  for(uint32_t i = 0; i < word_len; i++) {
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, data[i])) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  return true;
}


// bool flash_erase_boot_data(void) {
//   bool ret = true;
  
//   FLASH_EraseInitTypeDef erase_config;
//   uint32_t page_error;
//   erase_config.TypeErase    = FLASH_TYPEERASE_SECTORS;
//   erase_config.Sector       = 1;
//   erase_config.NbSectors    = 1;
//   erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  
//   HAL_FLASH_Unlock();
//   if (HAL_OK != HAL_FLASHEx_Erase(&erase_config, &page_error)) {
//     ret = false;
//   }
//   if (0xFFFFFFFF != page_error) {
//     ret = false;
//   }
//   FLASH_WaitForLastOperation(HAL_MAX_DELAY);
//   HAL_FLASH_Lock();
//   return ret;
// }

// bool flash_word_write(uint32_t addr, uint32_t *data, uint32_t word_len) {
//   HAL_FLASH_Unlock();
//   for(uint32_t i = 0; i < word_len; i++) {
//     if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, data[i])) {
//       HAL_FLASH_Lock();
//       return false;
//     }
//   }
//   HAL_FLASH_Lock();
//   return true;
// }

/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/

