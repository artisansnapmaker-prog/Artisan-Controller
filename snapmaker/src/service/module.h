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
#ifndef SNAPMAKER_MODULE_SERVICE_H_
#define SNAPMAKER_MODULE_SERVICE_H_

#include "../common/list.h"

#include "../host/sacp.h"
#include "../host/sm_mac.h"
#include "../host/sm_can.h"

#include "../module/base.h"

#define MODULE_ACCESSIBLE_MAX  (64)


enum ModuleServiceStatus {
  MS_STATUS_UNCONFIG,   // unconfigured
  MS_STATUS_ON_INIT,    // is initializing
  MS_STATUS_CONFIG,     // configured
  MS_STATUS_UPGRADING,  // is upgrading modules
  MS_STATUS_INVALID
};

typedef struct {
  uint16_t tail;
  uint16_t bound;
} message_id_record_t;

class ModuleService {
  // public methods
  public:
    ModuleService() {}

    int init();
    int register_routine(std::function <int(int)> routine);

    // background thread
    int background_thread();

  // private methods
  private:
    // callbacks
    int handle_module_inserted(uint32_t mac, uint8_t channel);
    int report_module_info(sacp_message_t &message);
    int handle_fw_request(sacp_message_t &message);

    // internal helper
    int init_virtual_modules();
    int get_function_list(ModuleBase &module, uint8_t channel);
    int record_function_list(ModuleBase &module, function_node_t *fnodes, uint8_t len);

    int assign_message_id();
    int bind_message_id();
    int bind_message_id(ModuleBase &module);

    // when controller receiver a whole module FW,
    // will tell us to upgrade modules
    int do_upgrade();

    void check_online();

  // private properties
  private:
    ModuleServiceStatus status = MS_STATUS_UNCONFIG;

    std::function <void(void)> routines[MODULE_ACCESSIBLE_MAX];

    ModuleBase *modules[MODULE_ACCESSIBLE_MAX];
    uint8_t    configured_module = 0;

    struct list_head function_list[MODULE_FUNC_PRIORITY_MAX];
    uint8_t max_function_one_module = 0;

    // record the range of used message id and the bound,
    // the id in one priority cannot reach its bound,
    // that is to say, bound of previous priority, is the first one
    // of the range of next priority.
    message_id_record_t msg_id_records[MODULE_FUNC_PRIORITY_MAX];
};

extern ModuleService module_svc;

#endif  // #ifndef SNAPMAKER_MODULE_SERVICE_H_
