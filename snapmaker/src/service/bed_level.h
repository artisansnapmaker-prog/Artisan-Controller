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
#ifndef SNAPMAKER_BED_LEVEL_SERVICE_H_
#define SNAPMAKER_BED_LEVEL_SERVICE_H_

#include "../config.h"
#include "motion.h"

#define CALIBRATION_PAPER_THICKNESS   0.0
class BedLevelService {
  public:
    BedLevelService() {
      z_compensation_[0] = 1.5;
      z_compensation_[1] = 1.5;
      live_z_offset_ = 0;
    }

    void init();
    err_code_t set_leveling_limit(float x_min, float x_max, float y_min, float y_max);
    err_code_t set_leveling_grids(uint8_t grids);
    err_code_t set_z_values(float z, uint8_t i, uint8_t j);
    err_code_t refresh_leveling_data();
    err_code_t set_live_z_offset(float offset);
    err_code_t start_probe_test(uint8_t b, float x, float y);
    err_code_t start_manual_bed_leveling(uint8_t grids);
    err_code_t goto_leveling_point(uint8_t index);
    err_code_t finish_manual_bed_leveling();
    err_code_t start_auto_bed_leveling(uint8_t grids);
    err_code_t probe_sensor_calibration(float x, float y);
    err_code_t confirm_probe_sensor_calibration(uint8_t e);
    err_code_t work_height_auto_detection();


    float z_values_[GRID_MAX_NUM][GRID_MAX_NUM];
    float z_compensation_[EXTRUDERS];
  private:
    float hotend_triggered_z_[EXTRUDERS];
    float hotend_touch_bed_z_[EXTRUDERS];
    uint8_t manual_leveling_point_index_;
    float manual_leveling_z_values_[GRID_MAX_NUM*GRID_MAX_NUM];
    float live_z_offset_;
};


extern BedLevelService bedlevel_svc;

#endif  // #ifndef SNAPMAKER_BED_LEVEL_SERVICE_H_
