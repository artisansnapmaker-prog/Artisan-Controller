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

#ifndef SNAPMAKER_UPGRADE_MODULE_INTERFACE_H_
#define SNAPMAKER_UPGRADE_MODULE_INTERFACE_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../src/config.h"
#include "../../common/error.h"
#include "../../common/ring_buffer.h"
#include "../../common/type.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"


typedef struct {
  uint32_t mac;
  uint32_t ch;
} module_info_t;

typedef err_code_t (*ugr_module_start_req )(pack_info_t *, module_info_t *);
typedef err_code_t (*ugr_module_start_ack)(uint8_t);
typedef err_code_t (*ugr_module_ready_req)(void);
typedef err_code_t (*ugr_module_ready_ack)(uint8_t ret);
typedef err_code_t (*ugr_module_start_trans)(void);
typedef err_code_t (*ugr_module_trans_req)(uint32_t req_offset, uint32_t len);
typedef err_code_t (*ugr_module_trans_ack)(uint32_t ack_offset, uint8_t *data, uint32_t len);
typedef err_code_t (*ugr_module_end_req)(uint8_t);
typedef err_code_t (*ugr_module_end_ack)(uint8_t);
typedef err_code_t (*ugr_module_notify_req)(uint8_t err_code);
typedef err_code_t (*ugr_module_notify_ack)(uint8_t err_code);

typedef struct {
  ugr_module_start_req start_req;
  ugr_module_start_ack start_ack;

  ugr_module_ready_req ready_req;
  ugr_module_ready_ack ready_ack;
  ugr_module_start_trans start_trans;
  
  ugr_module_trans_req trans_req;
  ugr_module_trans_ack trans_ack;
  
  ugr_module_end_req end_req;
  ugr_module_end_ack end_ack;
  
  ugr_module_notify_req notify_req;
  ugr_module_notify_ack notify_ack;
} UpgradeModuleHandle;

typedef err_code_t (*moudle_handle_init)(UpgradeModuleHandle *);
typedef void (*moudle_handle_deinit)(void);

typedef struct {
  UpdatePackType pack_type;
  uint16_t start_id;
  uint16_t end_id;
  moudle_handle_init module_init;
  moudle_handle_deinit module_deinit;
  UpgradeModuleHandle handle;
} UpgradeModuleInfo;


UpgradeModuleInfo *get_module_upgrade_handls(UpdatePackType pack_type, uint16_t id);


err_code_t module_call_start_ack(uint8_t ret);
err_code_t module_call_ready_ack(uint8_t ret);
err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len);
err_code_t module_call_end_ack(uint8_t ret);
err_code_t module_call_notify_req(uint8_t ret);

#endif  // #ifndef SNAPMAKER_UPGRADE_MODULE_INTERFACE_H_
