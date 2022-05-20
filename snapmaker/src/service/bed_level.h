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

// #include "../config.h"
// #include "motion_platform.h"
#include "../config.h"
#include "../module/toolhead_fdm.h"

#define CALIBRATION_PAPER_THICKNESS   0.1
#define LIVE_Z_OFFSET_LIMIT           0.5

/*************************************************************************************************************************************
reference links: https://snapmaker2.atlassian.net/wiki/spaces/SNAP/pages/1984824804/FDM?focusedCommentId=2010743286#comment-2010743286
*************************************************************************************************************************************/
typedef enum {
  BEDLEVEL_REQ_CMD_ID_SET_LEVEL_MODE           = 0x00,
  BEDLEVEL_REQ_CMD_ID_START_LEVEL              = 0x03,
  BEDLEVEL_REQ_CMD_ID_GOTO_PROBE_POINT         = 0x04,
  BEDLEVEL_REQ_CMD_ID_EXIT_LEVEL               = 0x06,
  BEDLEVEL_REQ_CMD_ID_GET_LEVEL_STATE          = 0x07,
  BEDLEVEL_REQ_CMD_ID_ABORT_AUTO_BEDLEVEL      = 0x09,
  BEDLEVEL_REQ_CMD_ID_BED_POSITION_DETECTION   = 0x12,
  BEDLEVEL_REQ_CMD_ID_PROBE_SENSOR_CALIBRATION = 0x13,
  BEDLEVEL_REQ_CMD_ID_SET_LIVE_Z_OFFSET        = 0x15,
  BEDLEVEL_REQ_CMD_ID_GET_LIVE_Z_OFFSET        = 0x16,

  BEDLEVEL_REQ_CMD_ID_SUM                      = 11,               // Adding or deleting IDs requires changing this value

  BEDLEVEL_CMD_ID_REPORT_BEDLEVEL_POINT        = 0xa1,

}bedlevel_req_cmd_id_e;

// level mode
#define BEDLEVEL_MODE_IDLE                    0
#define BEDLEVEL_MODE_AUTO                    2
#define BEDLEVEL_MODE_MANUAL                  3
#define BEDLEVEL_MODE_AUTO_BED_DETECTION      52
#define BEDLEVEL_MODE_MANUAL_BED_DETECTION    53
#define BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE  54
#define BEDLEVEL_MODE_XY_CALIBRATION          101

#define AUTO_PROBE_SENSOR_X_POSITION          214.4
#define AUTO_PROBE_SENSOR_Y_POSITION          244
#define AUTO_PROBE_SENSOR_Z_POSITION          20
#define AUTO_HOTEND_OFFSET_CALIBRATION_X_POSITION  214.4
#define AUTO_HOTEND_OFFSET_CALIBRATION_Y_POSITION  206
#define AUTO_HOTEND_OFFSET_CALIBRATION_Z_POSITION  13.8

#define BEDLEVEL_LIVE_Z_OFFSET_DEFAULT  0

typedef struct {
  float live_z_offset[EXTRUDERS];
} __attribute__((packed)) bedlevel_settings_t;

class BedLevelService {
  public:
    BedLevelService() {
      bedlevel_mode = BEDLEVEL_MODE_IDLE;
      z_compensation_[0] = 1.5;
      z_compensation_[1] = 1.5;
      live_z_offset_changed = false;
      need_to_abort_auto_bedlevel = false;
    }

    void init();
    err_code_t set_leveling_limit(float x_min, float x_max, float y_min, float y_max);
    err_code_t set_leveling_grids(uint8_t grids);
    err_code_t set_z_values(float z, uint8_t i, uint8_t j);
    err_code_t refresh_leveling_data();
    err_code_t start_probe_test(uint8_t b, float x, float y);
    err_code_t start_manual_bed_leveling(uint8_t grids);
    err_code_t goto_leveling_point(uint8_t index);
    err_code_t finish_manual_bed_leveling();
    err_code_t start_auto_bed_leveling(uint8_t grids);
    err_code_t probe_sensor_calibration(float x, float y);
    err_code_t confirm_probe_sensor_calibration(uint8_t e);
    err_code_t work_height_auto_detection();
    err_code_t set_bedlevel_mode(uint8_t mode);
    uint8_t get_bedlevel_mode();
    bool is_bedleveled();
    void set_end_leveling_process_status(bool status);
    bool get_end_leveling_process_status();
    err_code_t apply_live_z_offset(uint8_t e);
    err_code_t unapply_live_z_offset(uint8_t e);
    void set_live_z_offset(uint8_t e, float offset);
    void auto_probe_sensor_calibration();
    void auto_hotend_offset_calibration();
    void toolhead_auto_calibation();
    void update_soft_endstop_max_z();
    void report_probe_sensor_compensation();


    float z_values_[GRID_MAX_NUM][GRID_MAX_NUM];
    float z_compensation_[EXTRUDERS];
    float detected_bed_z_values[EXTRUDERS];
    float hotend_triggered_z_[EXTRUDERS];
    float hotend_touch_bed_z_[EXTRUDERS];
    float live_z_offset[EXTRUDERS];
    bool live_z_offset_changed;
    bool need_to_abort_auto_bedlevel;
  private:
    uint8_t bedlevel_mode;
    uint8_t manual_leveling_point_index_;
    uint8_t manual_leveling_point_sum;
    float manual_leveling_z_values_[GRID_MAX_NUM*GRID_MAX_NUM];
    bool is_bed_leveled;
    bool end_of_leveling_process;
};


extern BedLevelService bedlevel_svc;

#endif  // #ifndef SNAPMAKER_BED_LEVEL_SERVICE_H_
