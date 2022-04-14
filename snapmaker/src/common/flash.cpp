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
#include <Arduino.h>
#include "utility.h"
#include "flash.h"

const flash_sector_addr_t flash_sector_tab[] = {
  {FLASH_BASE + 0,                    16 * 1024},
  {FLASH_BASE + 16 * 1024,            16 * 1024},
  {FLASH_BASE + 32 * 1024,            16 * 1024},
  {FLASH_BASE + 48 * 1024,            16 * 1024},
  {FLASH_BASE + 64 * 1024,            64 * 1024},
  {FLASH_BASE + 128 * 1024,           128 * 1024},
  {FLASH_BASE + 256 * 1024,           128 * 1024},
  {FLASH_BASE + 384 * 1024,           128 * 1024},
  {FLASH_BASE + 512 * 1024,           128 * 1024},
  {FLASH_BASE + 640 * 1024,           128 * 1024},
  {FLASH_BASE + 768 * 1024,           128 * 1024},
  {FLASH_BASE + 896 * 1024,           128 * 1024},
};


flash_partition_t boot_data_partition = {
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};

flash_partition_t module_fw_partition = {
  FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Start addr
  FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Write addr
  FLASH_MODULE_FW_DOWNLOAD_SIZE      // Partition addr  
};

bool flash_addr_to_sector_number(uint32_t start, uint32_t size, uint32_t &sector_start, uint32_t &sector_number) {
  uint32_t ts = TAB_SIZE(flash_sector_tab, flash_sector_addr_t);

  if (start < flash_sector_tab[0].start)
    return false;
  if ((start + size) > 
      flash_sector_tab[ts - 1].start + 
      flash_sector_tab[ts - 1].size)
    return false;
  if (!size)
    return false;

  bool s, e;
  s = e = false;
  for (uint32_t i  = 0; i < ts; i++) {
    if (!s && start < flash_sector_tab[i].start) {
      sector_start = i - 1;
      s = true;
    }
    if (!s && start == flash_sector_tab[i].start) {
      sector_start = i;
      s = true;
    }
    if (!e && (start + size) <= flash_sector_tab[i].start) {
      sector_number = i - sector_start;
      e = true;
      break;
    }
  }

  if (!s) {
    sector_start = ts - 1;
  }

  if (!e) {
    sector_number = ts - sector_start;
  }

  return true;
}

/*************************************************************** Flash_Start ***************************************************************/
void flash_reset(flash_partition_t &flash_partition) {
  flash_partition.write_addr = flash_partition.start_addr;
}

bool flash_erase(flash_partition_t &flash_partition) {
  bool ret = true;
  FLASH_EraseInitTypeDef erase_config;
  uint32_t page_error;
  
  if (!flash_addr_to_sector_number( flash_partition.start_addr, flash_partition.size, 
                                    erase_config.Sector, erase_config.NbSectors))
    return false;

  HAL_FLASH_Unlock();
  if (HAL_OK != HAL_FLASHEx_Erase(&erase_config, &page_error)) {
    ret = false;
  }
  if (0xFFFFFFFF != page_error) {
    ret = false;
  }
  FLASH_WaitForLastOperation(HAL_MAX_DELAY);
  HAL_FLASH_Lock();

  flash_partition.write_addr = flash_partition.start_addr;
  return ret;
}

uint32_t flash_write(flash_partition_t &flash_partition, uint8_t *data, uint32_t len) {
  uint32_t need_to_write_len;
  uint32_t flash_free_size;
  uint32_t have_write_len;

  if(!len) {
    return 0;
  }
  
  if (flash_partition.write_addr < flash_partition.start_addr) {
    flash_free_size = 0;
  }
  else {
    flash_free_size = flash_partition.size - (flash_partition.write_addr - flash_partition.start_addr);
  }

  if (0 == flash_free_size) {
    return 0;
  }

  have_write_len = 0;
  need_to_write_len = MIN(flash_free_size, len);
  
  HAL_FLASH_Unlock();
  // Write the not aligned data
  uint8_t *_8_data = data;
  // uint32_t _not_aligned_write_len = ROUNDUP(uint32_t(flash_partition.write_addr), 4) - (uint32_t)flash_partition.write_addr;
  // for(uint32_t i = 0; i < _not_aligned_write_len; i++) {
  for(uint32_t i = 0; i < need_to_write_len; i++) {
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_partition.write_addr, *_8_data)) {
      break;
    }
    else {
      flash_partition.write_addr += 1;
      have_write_len++;
      _8_data++;
    }
  }

  // // Write in 8 bytes for fast
  // uint64_t *_64_data = (uint64_t *)_8_not_aligned_data;
  // uint32_t _double_word_write_len = (need_to_write_len - _not_aligned_write_len)>>3;
  // for(uint32_t i = 0; i < _double_word_write_len; i++) {
  //   if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_partition.write_addr, *_64_data)) {
  //     break;
  //   }
  //   else {
  //     flash_partition.write_addr += 8;
  //     _64_data++;
  //   }
  // }

  // // Write the remain
  // uint8_t *_8_data = (uint8_t *)_64_data;
  // uint32_t _remain_write_len = (need_to_write_len - _not_aligned_write_len) & 0x00000007;
  // for(uint32_t i = 0; i < _remain_write_len; i++) {
  //   if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_partition.write_addr, *_8_data)) {
  //     break;
  //   }
  //   else {
  //     flash_partition.write_addr += 1;
  //     _8_data++;
  //   }
  // }
  HAL_FLASH_Lock();

  return have_write_len;
}

/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/

