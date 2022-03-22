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
#ifndef SNAPMAKER_MODULE_BED_MULTI_ZONE_H_
#define SNAPMAKER_MODULE_BED_MULTI_ZONE_H_

#include "base.h"

enum BedSacpRequestCommandId {
  SACP_CMD_ID_BED_GET_HEAD_INFO = 1,
  SACP_CMD_ID_BED_SET_TARGET_TEMP,

  // Fixed parameters, not modifiable
  SACP_CMD_ID_BED_END_INDEX,
  SACP_CMD_ID_BED_MAX_NUM = SACP_CMD_ID_BED_END_INDEX - 1,
};

#define SACP_BED_SUBSCRIBE_COMMANDID              0xa0

#pragma pack(1)
typedef struct {
  uint8_t bed_index; 
  int32_t cur_temp;
  uint16_t target_temp;
}ZoneInfo;

#pragma pack()


class BedVirtual: public ModuleBase {
  // public methods
  public:
    BedVirtual(uint8_t zone_number, uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {}
    bool check_online() { return true; }
    err_code_t pre_init() { return E_SUCCESS; }
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }
    err_code_t save_env(uint8_t *env_buf, uint32_t &len);
    err_code_t resume_env(uint8_t *env_buf, uint32_t &len);

    friend err_code_t send_bed_info_to_hmi(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_bed_target_temp(void *obj, sacp_hmi_message_t *msg);
    friend uint16_t hmi_subscribe_bed_func(void *obj, uint8_t *buff);

    void bed_hmi_self_test_interface(uint8_t test_type, uint32_t param);
  // private methods
  private:

  // public properties
  public:


  // private properties
  private:
    uint8_t zone_number;
};


#endif  // #ifndef SNAPMAKER_MODULE_BED_MULTI_ZONE_H_

