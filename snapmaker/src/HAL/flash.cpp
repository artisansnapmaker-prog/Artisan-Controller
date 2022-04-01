#include "flash.h"
#include "Arduino.h"
#include "../common/error.h"
#include "../common/debug.h"


err_code_t flash_erase_sector(int sector_num) {
  HAL_StatusTypeDef ret;

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
#if defined(STM32H7xx)
  EraseInitStruct.Banks = FLASH_BANK_NUMBER;
#endif
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = sector_num;
  EraseInitStruct.NbSectors = 1;

  ret = HAL_FLASH_Unlock();

  if (ret != HAL_OK) {
    return E_HARDWARE;
  }

  ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  if (ret == HAL_OK) {
    LOG_E("failed to erase flash sector[%d]\n", sector_num);
  }

  HAL_FLASH_Lock();

  return E_SUCCESS;
}


void flash_write_buffer(uint8_t *buffer, size_t length, uint32_t start_address) {
  HAL_StatusTypeDef ret;
#if defined (STM32F0xx) || defined (STM32F1xx) || defined (STM32F3xx) || \
    defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L0xx) || \
    defined (STM32L1xx) || defined (STM32L4xx) || defined (STM32WBxx)
  uint32_t pageError = 0;
  uint64_t data = 0;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined (STM32G4xx) || defined (STM32L4xx) || defined (STM32F1xx)
  EraseInitStruct.Banks = FLASH_BANK_NUMBER;
#endif
#if defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || \
    defined (STM32WBxx)
  EraseInitStruct.Page = FLASH_PAGE_NUMBER;
#else
  EraseInitStruct.PageAddress = FLASH_BASE_ADDRESS;
#endif
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASH_Unlock() == HAL_OK) {
#if defined(STM32L0xx)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | \
                           FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
#elif defined(STM32L1xx)
#if defined(FLASH_SR_RDERR)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR);
#else
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
#endif
#elif defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || \
      defined (STM32WBxx)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
#else
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
#endif
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK) {
      while (address <= address_end) {
#if defined(STM32L0xx) || defined(STM32L1xx)
        memcpy(&data, eeprom_buffer + offset, sizeof(uint32_t));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
          address += 4;
          offset += 4;
#else
        data = *((uint64_t *)((uint8_t *)eeprom_buffer + offset));

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK) {
          address += 8;
          offset += 8;
#endif
        } else {
          address = address_end + 1;
        }
      }
    }
    HAL_FLASH_Lock();
  }
#else

  HAL_FLASH_Unlock();

  for (size_t i = 0; i < length; i+=4) {
#if defined(STM32H7xx)
      /* 256 bits */
      memcpy(&data, eeprom_buffer + offset, 8 * sizeof(uint32_t));
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) == HAL_OK) {
        address += 32;
        offset += 32;
#else
      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address + i, *(uint32_t *)(buffer + i));
      if (ret != HAL_OK) {
        LOG_E("flash write error, ret[%u]\n", ret);
        break;
      }
#endif
    
  }
  HAL_FLASH_Lock();
#endif
  return;
}

