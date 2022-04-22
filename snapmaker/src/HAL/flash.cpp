#include "flash.h"
#include "Arduino.h"
#include "../common/error.h"
#include "../common/debug.h"


err_code_t flash_erase_sector(int sector_num) {
  HAL_StatusTypeDef ret;

  FLASH_EraseInitTypeDef erase_cfg;
  uint32_t sector_error = 0;

  /* ERASING page */
  erase_cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase_cfg.Sector = sector_num;
  erase_cfg.NbSectors = 1;

  ret = HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  if (ret != HAL_OK) {
    return E_HARDWARE;
  }

  ret = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
  if (ret == HAL_OK) {
    //LOG_E("failed to erase flash sector[%d]\n", sector_num);
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  FLASH_WaitForLastOperation(HAL_MAX_DELAY);
  HAL_FLASH_Lock();

  return ret;
}


size_t flash_write_buffer(uint8_t *buffer, size_t length, uint32_t start_address) {
  HAL_StatusTypeDef ret;
  size_t i = 0;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  for (i = 0; i < length; i+=4) {
      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address + i, *(uint32_t *)(buffer + i));
      if (ret != HAL_OK) {
        // LOG_E("flash write error, ret[%u]\n", ret);
        break;
      }
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  FLASH_WaitForLastOperation(HAL_MAX_DELAY);

  HAL_FLASH_Lock();

  return i;
}

