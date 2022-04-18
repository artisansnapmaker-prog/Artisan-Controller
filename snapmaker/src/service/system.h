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
#define EXCEPTION_OWNER_INVALID   (0xFFFF)
struct ExceptionNode {  
  uint16_t owner;
  uint8_t  state;
  uint32_t ban;
};

#define EXCEPTION_ACTION_PAUSE_WORK
#define EXCEPTION_ACTION_STOP_WORK
#define EXCEPTION_ACTION_DISABLE_POWER_MOTIVE
#define EXCEPTION_ACTION_DISABLE_POWER_8P_MOTOR
#define EXCEPTION_ACTION_DISABLE_POWER_8P_TOOLHEAD
#define EXCEPTION_ACTION_DISABLE_POWER_4P_ADDON
#define EXCEPTION_ACTION_DISABLE_POWER_BED
#define EXCEPTION_ACTION_DISABLE_POWER_HMI

#define EXCEPTION_BAN_MOVING          (0x00000001)
#define EXCEPTION_BAN_WORKING         (0x00000002)
#define EXCEPTION_BAN_HEATING_HOTEND  (0x00000004)
#define EXCEPTION_BAN_HEATING_BED     (0x00000008)
#define EXCEPTION_BAN_TURN_ON_LASER   (0x00000010)
#define EXCEPTION_BAN_TURN_ON_CNC     (0x00000020)


enum A400ControllerExceptionState {
  A400_CTRL_EXCEP_STA_NO_TOOLHEAD = 1,
  A400_CTRL_EXCEP_STA_NO_BED,
  A400_CTRL_EXCEP_STA_NO_LINEAR,
  A400_CTRL_EXCEP_STA_MISS_LINEAR,
  A400_CTRL_EXCEP_STA_OVERTEMP,
  A400_CTRL_EXCEP_STA_MISS_SETTINGS,
  A400_CTRL_EXCEP_STA_HOME_FAILED,
  A400_CTRL_EXCEP_STA_REPLACE_TOOLHEAD,
};

/*
#define POWER_DOMAIN_MOTIVE_POWER (0x1)
#define POWER_DOMAIN_8P_TOOLHEAD  (0x1<<1)
#define POWER_DOMAIN_8P_MOTOR     (0x1<<2)
#define POWER_DOMAIN_4P_ADDON     (0x1<<3)
#define POWER_DOMAIN_BED          (0x1<<4)
#define POWER_DOMAIN_HMI          (0x1<<5)
*/

class SystemService {
  // public methods
  public:
    SystemService() {}
    void init();
    void background_thread() { return ; }
    uint32_t millis(void);


    err_code_t raise_exception(uint16_t owner, uint8_t state, uint32_t actions, uint32_t ban = 0);
    err_code_t clear_exception(uint16_t owner, uint8_t state);
    void raise_exception_from_isr(uint16_t owner, uint8_t state, uint32_t actions, uint32_t ban = 0);

    static err_code_t hmi_cb_get_exceptions(void *obj, sacp_hmi_message_t *msg);
    
  // private methods
  private:
    uint32_t get_bans(uint8_t *buffer, uint32_t buff_len);
    void update_bans();
    uint32_t get_level(uint32_t ban);

  // public properties
  public:

  // private properties
  private:
    ExceptionNode nodes[EXCEPTION_STATIC_SIZE];
    uint32_t      bans;

    // if node above is not enough for save current exceptions
    // won't apply dynamic memory from heap for them
    ExceptionNode *dynamic_nodes;
};


extern SystemService system_svc;


#endif
