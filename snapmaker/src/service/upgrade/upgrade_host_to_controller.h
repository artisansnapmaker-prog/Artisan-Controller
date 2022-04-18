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

#ifndef SNAPMAKER_UPGRADE_HOST_TO_CONTROLLER_H_
#define SNAPMAKER_UPGRADE_HOST_TO_CONTROLLER_H_


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


enum UpgradeHCStatus {
  UPGRADE_HC_STATUS_IDLE = 0,
  UPGRADE_HC_STATUS_START,
  UPGRADE_HC_STATUS_TRANS,
  UPGRADE_HC_STATUS_END,
};

class UpdateService;

/************************************************************************/
// module upgrade class
/************************************************************************/
class UpgradeHostToController {
  public:
    UpgradeHostToController(){};
    err_code_t init(UpdateService *s);
    void loop(void);
    void reset_to_idle(void);
    
    err_code_t sacp_msg_proc(sacp_hmi_message_t *msg);
    err_code_t start_proc(sacp_hmi_message_t *msg);
    err_code_t trans_proc(sacp_hmi_message_t *msg);
    err_code_t end_proc(sacp_hmi_message_t *msg);

  private:
    void start_ack(sacp_hmi_message_t *msg, uint8_t ret);
    void trans_data_req(uint32_t offset, uint16_t len);
    void end_req(uint8_t ret);
    void error_notify(uint8_t ret);

    UpgradeHCStatus status;
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
};

extern UpgradeHostToController ugr_hc_svc;


#endif  // #ifndef SNAPMAKER_UPGRADE_MODULE_H_
