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

#ifndef SNAPMAKER_UPGRADE_CONTROLLER_TO_MODULE_H_
#define SNAPMAKER_UPGRADE_CONTROLLER_TO_MODULE_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../src/config.h"
#include "../../common/error.h"
#include "../../common/ring_buffer.h"
#include "../../common/type.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"
#include "upgrade_module_interface.h"
#include "../module.h"
#include "../../module/base.h"


#define UPGRADE_CM_TRANS_BUF_SIZE   (128)


enum ModuleCMStatus {
  UPGRADE_CM_STATUS_IDLE = 0,
  UPGRADE_CM_STATUS_START,
  UPGRADE_CM_STATUS_WAIT_FOR_READY,
  UPGRADE_CM_STATUS_TRANS,
  UPGRADE_CM_STATUS_END,
};

class UpdateService;

/************************************************************************/
// module upgrade class
/************************************************************************/
class UpgradeControllerToModule {

  public:
    UpgradeControllerToModule(){};
    err_code_t init(UpdateService *s);
    void loop(void);
    void reset_to_idle(void);
    err_code_t start(void);
    ModuleBase *get_next_module(void);

    err_code_t module_call_start_ack(uint8_t);
    err_code_t module_call_ready_ack(uint8_t);
    err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len);
    err_code_t module_call_end_ack(uint8_t);
    err_code_t module_call_notify_req(uint8_t);

  private:
    err_code_t start_a_module_upgrade(void);
    void ready_req(void);
    void start_trans(void);
    void trans_data_req(uint32_t offset, uint16_t len);
    void end_req(uint8_t ret);
    void error_notify(uint8_t ret);

    int module_index;
    ModuleBase *module;
    pack_info_t *pit;
    module_info_t module_info;

    ModuleCMStatus status;
    UpdateService *ugr_svc;

    uint32_t fw_flash_addr;
    uint32_t fw_lenght;
    uint32_t fw_checksum;
    uint16_t fw_id;
    uint32_t offset;
    uint32_t trans_len;

    uint32_t last_action_ms;
    uint32_t action_req_try;

    uint8_t end_ret;

    UpgradeModuleInfo *module_upgrade_info;
};

extern UpgradeControllerToModule ugr_cm_svc;


#endif  // #ifndef SNAPMAKER_UPGRADE_CONTROLLER_TO_MODULE_H_
