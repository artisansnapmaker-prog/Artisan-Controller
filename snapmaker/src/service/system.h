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
#ifndef SNAPMAKER_SYSTEM_SERVICE_H_
#define SNAPMAKER_SYSTEM_SERVICE_H_

#include <stdint.h>
#include "../config.h"
#include "../common/debug.h"
#include "../common/error.h"
#include "../host/sacp_hmi.h"

#define EXCEPTION_STATIC_SIZE     (64)
#define EXCEPTION_ISR_QUEUE_SIZE  (4)
#define EXCEPTION_OWNER_INVALID   (0xFFFF)
struct ExceptionNode {
  uint16_t owner;
  uint8_t  state;
  uint32_t ban;
};

struct ExceptionNodeISR {
  uint16_t owner;
  uint8_t  state;
  uint32_t ban;
  uint32_t actions;
};



#define EXCEP_ACT_DISABLE_POWER               (EXCEP_ACT_DISABLE_POWER_MOTIVE | \
                                                EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD | \
                                                EXCEP_ACT_DISABLE_POWER_8P_MOTOR | \
                                                EXCEP_ACT_DISABLE_POWER_4P_ADDON | \
                                                EXCEP_ACT_DISABLE_POWER_BED | \
                                                EXCEP_ACT_DISABLE_POWER_HMI)


#define EXCEP_BAN_CANNOT_WORK                 (EXCEP_BAN_ENABLE_POWER_MOTIVE | \
                                                    EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD | \
                                                    EXCEP_BAN_ENABLE_POWER_8P_MOTOR | \
                                                    EXCEP_BAN_ENABLE_POWER_4P_ADDON | \
                                                    EXCEP_BAN_ENABLE_POWER_HMI | \
                                                    EXCEP_BAN_ENABLE_POWER_BED | \
                                                    EXCEP_BAN_MOVING | \
                                                    EXCEP_BAN_WORKING)

class SystemService {
  // public methods
  public:
    SystemService() {}
    void init();
    void background_thread();
    uint32_t millis(void);

    uint32_t get_bans() { return bans; }

    bool allow_working();
    bool allow_moving();
    bool allow_heating_bed();
    bool allow_heating_hotend();
    bool allow_leveling();
    bool allow_turn_on_laser();
    bool allow_turn_on_cnc();
    /* raise exception from thread env
    *  owner   - device id
    *  state   - exception enumeration, each owner must define itself exception
    *  actions - actions you want to trigger, these have been define in system.h, 
    *            the macros start with prefix 'EXCEP_ACT_'
    *  ban     - when an exception exists, the behaviors you want to ban, 
    *            the macros start with prefix 'EXCEP_BAN_'
    */
    err_code_t raise_exception(uint16_t owner, uint8_t state, uint32_t actions = 0, uint32_t ban = 0);

    /* clear exception from thread env
    *  owner   - device id
    *  state   - exception enumeration, each owner must define itself exception
    */
    err_code_t clear_exception(uint16_t owner, uint8_t state);

    /* raise exception from thread env */
    void raise_exception_from_isr(uint16_t owner, uint8_t state, uint32_t actions = 0, uint32_t ban = 0);

    static err_code_t hmi_cb_get_exceptions(void *obj, sacp_hmi_message_t *msg);

  // private methods
  private:
    uint32_t get_bans(uint8_t *buffer, uint32_t buff_len);
    void update_bans();
    uint32_t get_level(uint32_t ban);
    void lock_nodes();
    void unlock_nodes();

  // public properties
  public:

  // private properties
  private:
    BaseType_t lock_sta = pdFAIL;
    SemaphoreHandle_t node_lock = NULL;
    ExceptionNode nodes[EXCEPTION_STATIC_SIZE];
    uint32_t      bans = 0;

    // if node above is not enough for save current exceptions
    // won't apply dynamic memory from heap for them
    ExceptionNode *dynamic_nodes;

    // queue to receive exception from ISR
    ExceptionNodeISR nodes_isr[EXCEPTION_ISR_QUEUE_SIZE];
};


extern SystemService system_svc;


#endif
