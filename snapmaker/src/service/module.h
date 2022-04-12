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
#include "../common/error.h"

#include "../host/sacp_module.h"
#include "../host/sacp_hmi.h"
#include "../host/sm_mac.h"
#include "../host/sm_can.h"

#include "../module/base.h"

#define MODULE_ACCESSIBLE_MAX  (64)


enum ModuleServiceStatus {
  MS_STATUS_UNCONFIG,   // unconfigured
  MS_STATUS_SCANNING,   // is scanning modules
  MS_STATUS_BINDING,    // is binding message id
  MS_STATUS_CONFIG,     // configured
  MS_STATUS_UPGRADING,  // is upgrading modules
  MS_STATUS_INVALID
};

typedef struct {
  uint16_t tail;
  uint16_t bound;
} message_id_record_t;

typedef err_code_t (*routine_function)(void *obj);

typedef struct {
  void *obj;
  routine_function cb;
} module_routine_t;

class ModuleService {
  // public methods
  public:
    ModuleService() {
      for (int i = 0; i < MODULE_ACCESSIBLE_MAX; i++) {
        routines[i].obj = NULL;
        routines[i].cb = NULL;
      }
    }

    void init();
    err_code_t register_routine(void *obj, routine_function cb);
    void unregister_routine(void *obj);

    ModuleBase *get_module(uint16_t device_id, uint8_t sub_index) {
      for (int i = 0; i < configured_module; i++) {
        if (!modules[i])
          continue;

        if (modules[i]->get_device_id() == device_id &&
          modules[i]->get_sub_index() == sub_index)
          return modules[i];
      }

      return NULL;
    }

    ModuleBase *get_module(int index) {
      if (index < MODULE_ACCESSIBLE_MAX)
        return modules[index];
      else
        return NULL;
    }


    // background thread
    void background_thread();

    static err_code_t handle_module_inserted(void *obj, uint32_t mac, LinkCANChannel ch);
    static err_code_t handle_fw_request(void *obj, sacp_module_message_t &message);
    static err_code_t report_module_info(void *obj, sacp_hmi_message_t *message);

    void standby_all();
    void quick_stop_all();
    void emergency_stop_all();
    void scan_modules();

  // private methods
  private:
    // callbacks

    // internal helper
    int init_virtual_modules();
    err_code_t get_function_list(ModuleBase &module);
    err_code_t record_function_list(ModuleBase &module, function_node_t *fnodes, uint8_t len);

    err_code_t assign_message_id();
    err_code_t bind_message_id();
    err_code_t bind_message_id(ModuleBase &module);
    err_code_t get_module_info(ModuleBase &module);

    // when controller receiver a whole module FW,
    // will tell us to upgrade modules
    int do_upgrade();

    void check_online();

  // private properties
  private:
    ModuleServiceStatus status = MS_STATUS_UNCONFIG;

    module_routine_t routines[MODULE_ACCESSIBLE_MAX];

    ModuleBase *modules[MODULE_ACCESSIBLE_MAX];
    uint8_t    configured_module = 0;
    SemaphoreHandle_t configuring_lock = NULL;

    struct list_head function_list[MODULE_FUNC_PRIORITY_MAX];
    uint8_t max_function_one_module = 0;

    // record the range of used message id and the bound,
    // the id in one priority cannot reach its bound,
    // that is to say, bound of previous priority, is the first one
    // of the range of next priority.
    message_id_record_t msg_id_records[MODULE_FUNC_PRIORITY_MAX];

    uint32_t next_ms_background_broadcast;
};

extern ModuleService module_svc;

#endif  // #ifndef SNAPMAKER_MODULE_SERVICE_H_
