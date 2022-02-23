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
#ifndef SNAPMAKER_TOOLHEAD_CNC_H_
#define SNAPMAKER_TOOLHEAD_CNC_H_

#include "base.h"

#define CNC_POWER_MAX (100)
#define CNC_LOST_MAX  (6)

enum CNCOutputStatus {
  CNC_OUTPUT_OFF = 0,
  CNC_OUTPUT_ON = 1,

  CNC_OUTPUT_INVALID
};

class ToolHeadCNC: public ModuleBase {
  public:
		ToolHeadCNC(uint32_t mac, uint8_t key): ModuleBase(mac, key) {
      power     = 0;
      rpm       = 0;
    }

    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit();

    bool check_online() { return online; }

        uint16_t get_rpm() { return rpm; }
    void set_rpm(uint16_t new_rpm) { rpm = new_rpm; }

    uint8_t get_power() { return power; }

    void set_power(uint8_t new_power) {
      if (new_power > CNC_POWER_MAX)
        new_power = CNC_POWER_MAX;
      power = new_power;
    }

    void set_output(uint8_t new_power);

    void turn_on();
    void turn_off();

    void keep_alive() { lost_counter = 0; }

    friend err_code_t cnc_callback_routine(void *obj);
    friend void cnc_callback_update_rpm(void *obj, uint8_t *data, uint8_t length);

  private:
    err_code_t sync_power(uint8_t power);

  private:
    CNCOutputStatus output_sta;
    uint8_t  power;
    uint16_t rpm;
    uint8_t lost_counter = 0;
    bool  online = false;
};


#endif  // #ifndef TOOLHEAD_LASER_H_
