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

#ifndef SNAPMAKER_UPGRADE_HOST_TO_MODULE_H_
#define SNAPMAKER_UPGRADE_HOST_TO_MODULE_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../src/config.h"
#include "../../common/error.h"
#include "../../common/ring_buffer.h"
#include "../../common/type.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"


#define UPGRADE_TRANS_BUF_SIZE                (SACP_PDU_MAX_SIZE - 64)

enum UpgradeHMStatus {
  UPGRADE_HM_STATUS_IDLE = 0,
  UPGRADE_HM_STATUS_START,
  UPGRADE_HM_STATUS_TRANS,
  UPGRADE_HM_STATUS_END,
};

typedef err_code_t (*ugr_module_start_req )(pack_info_t *);
typedef err_code_t (*ugr_module_start_ack)(uint8_t);
typedef err_code_t (*ugr_module_trans_req)(uint32_t req_offset, uint32_t len);
typedef err_code_t (*ugr_module_trans_ack)(uint32_t ack_offset, uint8_t *data, uint32_t len);
typedef err_code_t (*ugr_module_end_req)(uint8_t);
typedef err_code_t (*ugr_module_end_ack)(uint8_t);
typedef err_code_t (*ugr_module_notify_req)(uint8_t err_code);
typedef err_code_t (*ugr_module_notify_ack)(uint8_t err_code);

typedef struct {
  ugr_module_start_req start_req;
  ugr_module_start_ack start_ack;
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

class UpdateService;

/************************************************************************/
// module upgrade class
/************************************************************************/

err_code_t module_call_start_ack(uint8_t);
err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len);
err_code_t module_call_end_ack(uint8_t);
err_code_t module_call_notify_req(uint8_t);

class UpgradeHostToModule {
  public:
    UpgradeHostToModule(){};
    err_code_t init(UpdateService *s);
    void loop(void);
    void reset_to_idle(void);
    
    err_code_t sacp_msg_proc(sacp_hmi_message_t *msg);
    err_code_t start_proc(sacp_hmi_message_t *msg);
    err_code_t trans_proc(sacp_hmi_message_t *msg);
    err_code_t end_proc(sacp_hmi_message_t *msg);

    err_code_t module_call_start_ack(uint8_t);
    err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len);
    err_code_t module_call_end_ack(uint8_t);
    err_code_t module_call_notify_req(uint8_t);

    UpgradeModuleInfo *get_module_upgrade_handls(UpdatePackType pack_type, uint16_t id);

  private:
    void trans_data_req(uint32_t offset, uint16_t len);
    void notify_end(uint8_t ret);
    void notify_error(uint8_t ret);
    bool firmware_flash_checksum(uint32_t rx_checsum, uint32_t flash_addr, uint32_t len);

    UpgradeHMStatus status;
    UpdateService *ugr_svc;

    uint32_t host_id;
    uint32_t host_ch;
    
    uint32_t offset;
    uint32_t fw_lenght;
    uint32_t checksum;

    uint32_t last_start_req_ms;
    uint32_t start_req_try;

    uint32_t last_trans_req_ms;
    uint32_t trans_req_try;

    uint32_t last_end_req_ms;
    uint32_t end_req_try;
    uint8_t end_ret;

    UpgradeModuleInfo *module_upgrade_info;
};

extern UpgradeHostToModule ugr_hm_svc;


#endif  // #ifndef SNAPMAKER_UPGRADE_MODULE_H_
