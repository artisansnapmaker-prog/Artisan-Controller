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
#ifndef SNAPMAKER_BOOT_H_
#define SNAPMAKER_BOOT_H_

#include <stdint.h>
#include <stdbool.h>
#include "../common/utility.h"
#include "../common/flash.h"

#define BOOT_DELAY_SECODE                       (6)
#define BOOT_PACK_MAGIC_STR_LEN                 (21)
#define BOOT_PACK_FW_VER_STR_LEN                (32)
#define BOOT_PACK_TIMESTAMP_STR_LEN             (20)
#define BOOT_DATA_DEFAULT_MAGIC_STR             ("snapmaker update.bin")
#define BOOT_DATA_DEFAULT_PROTOCOL_VER          (1)
#define BOOT_DATA_DEFAULT_PACK_TYPE             (A400_CONTROLLER_FW)
#define BOOT_DATA_DEFAULT_UPGRADE_FLAG          (UPGRADE_FLAG_ALWAYS)
#define BOOT_DATA_DEFAULT_UPGRADE_FLAG          (UPGRADE_FLAG_ALWAYS)
#define BOOT_DATA_DEFAULT_START_INDEX           (0)
#define BOOT_DATA_DEFAULT_END_INDEX             (0)
#define BOOT_DATA_DEFAULT_END_INDEX             (0)
#define BOOT_INFO_SIZE                          (256)

enum UpdateCtrlFlag {
  UPGRADE_CTRL_FLAG_NORMAL = 0,
  UPGRADE_CTRL_FLAG_ALWAYS = 1,
};

enum UpdateState {
  UPGRADE_STATE_FACTOR_BURN = 0xAA00,
  UPGRADE_STATE_WAIT = 0xAA01,
  UPGRADE_STATE_START = 0xAA02,
  UPGRADE_STATE_TRANS = 0xAA03,
  UPGRADE_STATE_END = 0xAA04,
  UPGRADE_STATE_JUMP_SUCCESS = 0xAA05,
};

enum UpdatePackType{
  SM2_CONTROLLER_FW = 1,
  A400_CONTROLLER_FW = 2,
  J1_CONTROLLER_FW = 3,
  SM2_MODULE_FW = 4,
  ESP32_FW = 5,
};

typedef enum {
  LINK_CH_PC = 0,
  LINK_CH_SC = 1,
} link_ch_e;

#pragma pack(1)
typedef struct{
  uint8_t magic_str[BOOT_PACK_MAGIC_STR_LEN];
  uint8_t protocol_ver;
  uint16_t pack_type;
  uint8_t upgrade_ctrl_flag;
  uint16_t start_index;
  uint16_t end_index;
  uint8_t fw_ver_str[BOOT_PACK_FW_VER_STR_LEN];
  uint8_t timestamp_str[BOOT_PACK_TIMESTAMP_STR_LEN];
  uint16_t upgrade_state;
  uint32_t fw_lenght;
  uint32_t fw_checksum;
  uint32_t fw_runaddr;
  uint8_t peer;
  uint8_t link_ch;
  uint32_t boot_data_checksum; 
} pack_info_t;
#pragma pack()

typedef void (*pf)(void);

static inline void load_boot_info(pack_info_t *pi) {
  memcpy(pi, (void *)FLASH_BOOT_DATA_ADDR, sizeof(pack_info_t));
}

static inline bool boot_info_check(pack_info_t *pti) {
  uint8_t *p = (uint8_t *)BOOT_DATA_DEFAULT_MAGIC_STR;
  for (uint32_t i = 0; i < BOOT_PACK_MAGIC_STR_LEN; i++) {
    if (pti->magic_str[i] != p[i])
      return false;
  }
  return pti->boot_data_checksum == calculate_checksum((uint8_t *)pti, sizeof(pack_info_t) - 4);
}

void setup(void);
void loop(void);
void print_boot_info(pack_info_t *pi);
bool application_fw_valid(uint32_t checksum, uint8_t *app_fw_start, uint32_t app_fw_len);
bool set_boot_upgrade_state_and_flush_to_flash(UpdateState s);
bool boot_info_flush_to_flash(void);
size_t send(link_ch_e ch, uint8_t *buf, uint32_t len);


#endif  // SNAPMAKER_BOOT_H_
