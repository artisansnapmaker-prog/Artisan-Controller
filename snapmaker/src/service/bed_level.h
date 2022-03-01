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

class BedLevelService {
  public:
    BedLevelService() {
      z_compensation_ = 1.5;
    }

    void init();
    err_code_t start_bed_level(uint8_t grids);

  private:
    float z_values_[GRID_MAX_NUM][GRID_MAX_NUM];
    float z_compensation_;
};


extern BedLevelService bedlevel_svc;

#endif  // #ifndef SNAPMAKER_BED_LEVEL_SERVICE_H_
