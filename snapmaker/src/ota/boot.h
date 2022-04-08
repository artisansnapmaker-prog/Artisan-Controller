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

#define BOOT_MODE_FACTORY_BURNING     (0xAA00)
#define BOOT_MODE_OTA_START           (0xAA01)
#define BOOT_MODE_OTA_TRANS           (0xAA02)
#define BOOT_MODE_OTA_RECV_DONE       (0xAA03)
#define BOOT_MODE_APP                 (0xAA04)
#define BOOT_DELAY_SECODE             (6)

typedef enum {
  LINK_CH_PC = 0,
  LINK_CH_SC = 1,
} link_ch_e;

#pragma pack(1)
typedef struct{
  uint8_t magic_str[21];
  uint8_t protocol_ver;
  uint16_t pack_type;
  uint8_t update_flag;
  uint16_t start_index;
  uint16_t end_index;
  uint8_t fw_ver_str[32];
  uint8_t timestamp_str[20];
  uint16_t boot_mode;
  uint32_t fw_lenght;
  uint32_t fw_checksum;
  uint32_t fw_runaddr;
  uint8_t peer;
  uint8_t link_ch;
  uint32_t boot_data_checksum; 
} boot_info_t;
#pragma pack()

typedef void (*pf)(void);

void print_boot_info(boot_info_t *bi);
void setup(void);
void boot_app(void);
bool boot_info_check(boot_info_t *bi);
void loop(void);
size_t send(link_ch_e ch, uint8_t *buf, uint32_t len);


#endif  // SNAPMAKER_BOOT_H_
