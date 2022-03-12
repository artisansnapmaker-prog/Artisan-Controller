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
#ifndef SNAPMAKER_DRYBOX_H_
#define SNAPMAKER_DRYBOX_H_

#include "base.h"

class DryBox: public ModuleBase {
  // public methods
  public:
    // construtor to do pre-init
    DryBox(uint32_t mac, uint8_t key):
    ModuleBase(mac, key) {

    }

    bool check_online() { return false; }
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }
    void update_temp_humidity(uint8_t *data);
    err_code_t set_fan_speed(uint8_t speed, uint8_t delay_time);
    err_code_t set_temp(int16_t heater_temp, int16_t chamber_temp);
    err_code_t set_pid(float p, float i, float d);
    err_code_t get_pid();
    void report_pid(uint8_t *data);

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    uint16_t device_id_;
    int16_t heater_temp_;
    int16_t chamber_temp_;
    uint16_t chamber_humidity_;

};

#endif  // #ifndef SNAPMAKER_DRYBOX_H_

