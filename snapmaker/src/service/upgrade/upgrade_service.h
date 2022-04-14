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

#ifndef SNAPMAKER_UPGRADE_H_
#define SNAPMAKER_UPGRADE_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../src/config.h"
#include "../../common/error.h"
#include "../../common/ring_buffer.h"
#include "../../common/type.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"
#include "upgrade_controller.h"
#include "upgrade_module.h"

#define CMD_SET_UPGRADE                       (0xAD)
#define CMD_ID_UPGRADE_START                  (0x01)
#define CMD_ID_UPGRADE_TRANS                  (0x02)
#define CMD_ID_UPGRADE_END                    (0x03)
#define CMD_ID_UPGRADE_NOTIFY                 (0x10)
#define CMD_ID_UPGRADE_NOTIFY                 (0x10)
#define SUB_ID_UPGRADE_STATUE

#define CMD_UPGRADE_START_MIN_LEN             (256)
#define SCAP_PAYLOAD_ADDITION_LEN             (8)
#define CMD_START_MIN_LEN                     (CMD_UPGRADE_START_MIN_LEN + SCAP_PAYLOAD_ADDITION_LEN)

enum UpgradeStatus {
  UPGRADE_INIT = 0,
  UPGRADE_CONTROLLER_TO_MODULE,
  UPGRADE_HOST_TO_CONTROLLER,
};


class UpgradeCtrlService;
class UpgradeModuleService;

class UpdateService {

  public:
    UpdateService(){};
    err_code_t init(void);
    void mark_boot_info(void);
    void loop(void);
    static err_code_t sacp_upgrade_start(void *obj, sacp_hmi_message_t *);
    err_code_t upgrade_start_ack(sacp_hmi_message_t *msg, err_code_t ret);

    bool boot_info_flush_to_flash();
    void set_boot_info(boot_info_t *bti);
    void print_boot_info(void);

    err_code_t controller_update_proc(sacp_hmi_message_t *msg);
    err_code_t module_update_proc(sacp_hmi_message_t *msg);

  private:
    void host_to_controller_loop(void);

    UpgradeStatus status;
    ModuleUpgradeType module_upgrade_type;
    boot_info_t boot_info;
    UpgradeCtrlService ctrl_ugr;
    UpgradeModuleService module_ugr;
};

extern UpdateService upgrade_svc;

#endif  // #ifndef SNAPMAKER_CLIENT_NODE_H_
