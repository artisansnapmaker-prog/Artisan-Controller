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
#ifndef SNAPMAKER_MODULE_LINEAR_VIRTUAL_H_
#define SNAPMAKER_MODULE_LINEAR_VIRTUAL_H_

#include "base.h"

enum LinearSACPCommandId {
  SACP_CMD_ID_LINEAR_GET_INFO = 1,
  SACP_CMD_ID_LINEAR_SET_ENDSTOP,

  SACP_CMD_ID_LINEAR_MAX = SACP_CMD_ID_LINEAR_SET_ENDSTOP
};

#define LINEAR_VIRTUAL_OBJECT_MAX (6)

extern pin_t X_MIN_PIN_var;
extern pin_t Y_MAX_PIN_var;
extern pin_t Y2_MAX_PIN_var;
extern pin_t Z_MAX_PIN_var;
extern pin_t Z2_MAX_PIN_var;
class LinearVirtual: public ModuleBase {
  // public methods
  public:
    LinearVirtual(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {
      if (object_index < LINEAR_VIRTUAL_OBJECT_MAX)
        objects[object_index++] = this;

      // TODO: setup detect pin
      switch (sub_index) {
      case MODULE_LINEAR_X1:
        endstop_pin = X_MIN_PIN_var;
        lead = 40;
        break;      

      case MODULE_LINEAR_Y1:
        endstop_pin = Y_MAX_PIN_var;
        lead = 40;
        break;      

      case MODULE_LINEAR_Z1:
        endstop_pin = Z_MAX_PIN_var;
        lead = 8;
        break;      

      case MODULE_LINEAR_Z2:
        endstop_pin = Z2_MAX_PIN_var;
        lead = 8;
        break;      

      case MODULE_LINEAR_Y2:
        endstop_pin = Y2_MAX_PIN_var;
        lead = 40;
        break;      

      default:
        break;      
      }
    }
    bool check_online() { return true; }
    err_code_t pre_init() { return E_SUCCESS; }
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }

    static err_code_t hmi_cb_get_info(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_endstop(void *obj, sacp_hmi_message_t *message);

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    static LinearVirtual *objects[LINEAR_VIRTUAL_OBJECT_MAX];
    static uint8_t object_index;
    uint32_t endstop_pin;
    uint32_t detect_pin;
    float lead;

};

#endif  // #ifndef SNAPMAKER_MODULE_LINEAR_VIRTUAL_H_
