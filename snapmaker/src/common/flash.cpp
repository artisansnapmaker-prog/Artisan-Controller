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

#include "utility.h"
#include "flash.h"


flash_partition_t boot_data_partition = {
  {
    FLASH_TYPEERASE_SECTORS,          // Mass erase or sector erase
    0,                                // Select banks to erase when Mass erase is enabled
    FLASH_BOOT_DATA_START_SECTOR,     // Initial FLASH sector to erase
    FLASH_BOOT_DATA_SECTOR_NUM,       // Number of sectors to be erased
    FLASH_VOLTAGE_RANGE_3             // The device voltage range
  },
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};

#if 0
flash_partition_t boot_data_partition = {
  {
    FLASH_TYPEERASE_SECTORS,          // Mass erase or sector erase
    0,                                // Select banks to erase when Mass erase is enabled
    FLASH_BOOT_DATA_START_SECTOR,     // Initial FLASH sector to erase
    FLASH_BOOT_DATA_SECTOR_NUM,       // Number of sectors to be erased
    FLASH_VOLTAGE_RANGE_3             // The device voltage range
  },
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};

flash_partition_t boot_data_partition = {
  {
    FLASH_TYPEERASE_SECTORS,          // Mass erase or sector erase
    0,                                // Select banks to erase when Mass erase is enabled
    FLASH_BOOT_DATA_START_SECTOR,     // Initial FLASH sector to erase
    FLASH_BOOT_DATA_SECTOR_NUM,       // Number of sectors to be erased
    FLASH_VOLTAGE_RANGE_3             // The device voltage range
  },
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};

flash_partition_t boot_data_partition = {
  {
    FLASH_TYPEERASE_SECTORS,          // Mass erase or sector erase
    0,                                // Select banks to erase when Mass erase is enabled
    FLASH_BOOT_DATA_START_SECTOR,     // Initial FLASH sector to erase
    FLASH_BOOT_DATA_SECTOR_NUM,       // Number of sectors to be erased
    FLASH_VOLTAGE_RANGE_3             // The device voltage range
  },
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};

flash_partition_t boot_data_partition = {
  {
    FLASH_TYPEERASE_SECTORS,          // Mass erase or sector erase
    0,                                // Select banks to erase when Mass erase is enabled
    FLASH_BOOT_DATA_START_SECTOR,     // Initial FLASH sector to erase
    FLASH_BOOT_DATA_SECTOR_NUM,       // Number of sectors to be erased
    FLASH_VOLTAGE_RANGE_3             // The device voltage range
  },
  FLASH_BOOT_DATA_ADDR,               // Start addr
  FLASH_BOOT_DATA_ADDR,               // Write addr
  FLASH_BOOT_DATA_SIZE                // Partition addr
};
#endif 

/*************************************************************** Flash_Start ***************************************************************/
bool flash_erase(flash_partition_t &flash_partition) {
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

uint32_t flash_write(flash_partition_t &flash_partition, uint8_t *data, uint32_t len) {
  uint32_t need_to_write_len;
  uint32_t flash_free_size;
  uint32_t have_write_len;

  if(!len) {
    return 0;
  }
  
  if (flash_partition.write_addr < flash_partition.start_addr) {
    flash_free_size = flash_partition.size;
    flash_partition.write_addr = flash_partition.start_addr;
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

