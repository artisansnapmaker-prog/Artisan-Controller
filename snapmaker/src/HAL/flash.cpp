#include "flash.h"
#include "Arduino.h"



void flash_write_buffer(uint8_t *buffer, size_t length, bool erase/*= true*/) {
//   FLASH_EraseInitTypeDef EraseInitStruct;
//   uint32_t offset = 0;
//   uint32_t address = FLASH_BASE_ADDRESS;
//   uint32_t address_end = FLASH_BASE_ADDRESS + E2END;
// #if defined (STM32F0xx) || defined (STM32F1xx) || defined (STM32F3xx) || \
//     defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L0xx) || \
//     defined (STM32L1xx) || defined (STM32L4xx) || defined (STM32WBxx)
//   uint32_t pageError = 0;
//   uint64_t data = 0;

//   /* ERASING page */
//   EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
// #if defined (STM32G4xx) || defined (STM32L4xx) || defined (STM32F1xx)
//   EraseInitStruct.Banks = FLASH_BANK_NUMBER;
// #endif
// #if defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || \
//     defined (STM32WBxx)
//   EraseInitStruct.Page = FLASH_PAGE_NUMBER;
// #else
//   EraseInitStruct.PageAddress = FLASH_BASE_ADDRESS;
// #endif
//   EraseInitStruct.NbPages = 1;

//   if (HAL_FLASH_Unlock() == HAL_OK) {
// #if defined(STM32L0xx)
//     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
//                            FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | \
//                            FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
// #elif defined(STM32L1xx)
// #if defined(FLASH_SR_RDERR)
//     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
//                            FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR);
// #else
//     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
//                            FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
// #endif
// #elif defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || \
//       defined (STM32WBxx)
//     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
// #else
//     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
// #endif
//     if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK) {
//       while (address <= address_end) {
// #if defined(STM32L0xx) || defined(STM32L1xx)
//         memcpy(&data, eeprom_buffer + offset, sizeof(uint32_t));
//         if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
//           address += 4;
//           offset += 4;
// #else
//         data = *((uint64_t *)((uint8_t *)eeprom_buffer + offset));

//         if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK) {
//           address += 8;
//           offset += 8;
// #endif
//         } else {
//           address = address_end + 1;
//         }
//       }
//     }
//     HAL_FLASH_Lock();
//   }
// #else
//   uint32_t SectorError = 0;
// #if defined(STM32H7xx)
//   uint64_t data[4] = {0x0000};
// #else
//   uint32_t data = 0;
// #endif

//   /* ERASING page */
//   EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//   EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//   EraseInitStruct.Sector = FLASH_DATA_SECTOR;
//   EraseInitStruct.NbSectors = 1;

//   HAL_FLASH_Unlock();

//   if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
//     while (address <= address_end) {
// #if defined(STM32H7xx)
//       /* 256 bits */
//       memcpy(&data, eeprom_buffer + offset, 8 * sizeof(uint32_t));
//       if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) == HAL_OK) {
//         address += 32;
//         offset += 32;
// #else
//       memcpy(&data, eeprom_buffer + offset, sizeof(uint32_t));
//       if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
//         address += 4;
//         offset += 4;
// #endif
//       } else {
//         address = address_end + 1;
//       }
//     }
//   }
//   HAL_FLASH_Lock();
// #endif
  return;
}

