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
#ifndef SNAPMAKER_ROTARY_H_
#define SNAPMAKER_ROTARY_H_

#include "base.h"

typedef struct {
  float position;
} __attribute__((packed)) rotary_recovery_data_t;

class Rotary: public ModuleBase {
  // public methods
  public:
    // construtor to do pre-init
    Rotary(uint32_t mac, uint8_t key, uint8_t sub_index):
    ModuleBase(mac, key, sub_index) {
    }

    bool check_online() { return false; }
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }
    err_code_t save_env(uint8_t *env_buf, uint32_t &len);
    err_code_t resume_env(uint8_t *env_buf, uint32_t &len);

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    uint16_t device_id;
};

#endif  // #ifndef SNAPMAKER_ROTARY_H_

