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
#ifndef SNAPMAKER_MOTION_SERVICE_H_
#define SNAPMAKER_MOTION_SERVICE_H_

#include "../config.h"

// X Y Z I J E0 E1
#define AXIS_NUM  7


class MotionService {
  public:
    MotionService();

    void init();

    // moving API
    void moveto_xy(float x, float y, float feedrate, bool blocked=true) {}
    void moveto_xyz(float x, float y, float z, float feedrate, bool blocked=true) {}
    void moveto_xyze(float x, float y, float z, float e, float feedrate, bool blocked=true) {}
    void moveto_x(float x, float feedrate, bool blocked=true) {}
    void moveto_y(float y, float feedrate, bool blocked=true) {}
    void moveto_z(float z, float feedrate, bool blocked=true) {}
    void moveto_a(float a, float feedrate, bool blocked=true) {}
    void moveto_b(float b, float feedrate, bool blocked=true) {}
    void moveto_e(float e, float feedrate, bool blocked=true) {}
    void moveto_e(float e, uint8_t extruder, float feedrate, bool blocked=true) {}
    void moveto(float target[AXIS_NUM], float feedrate, bool blocked=true);

    // position info API
    float current_position[AXIS_NUM];
    void  update_position() {}
    float get_current_position(uint8_t axis) {
      update_position();
      return current_position[axis];
    }

    // moving mode API
    void set_relative_mode(bool mode) {}
    bool get_relative_mode() { return false; }

    // speed control API
    float get_current_feedrate() { return 0.0; }
    void set_feedrate_percentage(int16_t percentage) {}

    // bed leveling API for internal app
    bool leveling_active() { return true; }
    void disable_leveling() {}
    void enable_leveling() {}

    // extruder control API
    uint8_t active_extruder() { return 0; }

    // temperature API
    float current_hotend_temp(uint8_t heater_id = 0) { return 0.0; }
    int16_t target_hotend_temp(uint8_t heater_id = 0) { return 0.0; }

    float current_bed_temp(uint8_t area_id = 0) { return 0.0; }
    int16_t target_bed_temp(uint8_t area_id = 0) { return 0.0; }

    bool runout_state(uint8_t extruder = 0) { return false; }

    // settings control
    void reset_settings() {}
    void load_settings() {}
    void save_settings() {}

    void run_gcode(char *gcode) {}

  private:
    TaskFunction_t thandle_marlin;

};


extern MotionService motion_svc;

#endif  // #ifndef SNAPMAKER_MOTION_SERVICE_H_
