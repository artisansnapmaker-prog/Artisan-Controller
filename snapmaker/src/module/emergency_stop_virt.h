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
#ifndef SNAPMAKER_MODULE_EMERGENCY_STOP_VIRTUAL_H_
#define SNAPMAKER_MODULE_EMERGENCY_STOP_VIRTUAL_H_

#include "base.h"
#include "../snapmaker.h"
class EmergencyStopVirtual: public ModuleBase {
  // public methods
  public:
    EmergencyStopVirtual(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {
      if (smprinter.get_model() == SNAPMAKER_MODEL_A400) {
        stop_button = PE1;
      }
    }
    bool check_online() { return true; }
    err_code_t pre_init() { return E_SUCCESS; }
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }

  static void show_info();

  // private methods
  private:

  // public properties
  public:

  // private properties
  private:
    static int16_t stop_button;
};

#endif  // #ifndef SNAPMAKER_MODULE_LINEAR_VIRTUAL_H_
