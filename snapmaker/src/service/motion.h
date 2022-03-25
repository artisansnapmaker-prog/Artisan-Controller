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
#include "module.h"
#include "../../../Marlin/src/module/temperature.h"
#include "../../../Marlin/src/module/planner.h"
#include "../../../Marlin/src/feature/bedlevel/bedlevel.h"
#include "../../Marlin/src/module/motion.h"
#include "../../Marlin/src/module/endstops.h"
#include "../../Marlin/src/module/probe.h"
#include "../../Marlin/src/module/tool_change.h"
#include "../../Marlin/src/module/settings.h"
#include "../common/error.h"
#include "src/gcode/gcode.h"


// X Y Z I J K
#define AXIS_NUM  6
#define XYZ 3

#define MOTION_PLATFORM_QUEUE_SIZE  (512)

enum AxisKey {
  AXIS_KEY_X1,
  AXIS_KEY_Y1,
  AXIS_KEY_Z1,
  AXIS_KEY_A1,
  AXIS_KEY_B1,
  AXIS_KEY_C1,
  AXIS_KEY_X2,
  AXIS_KEY_Y2,
  AXIS_KEY_Z2,
  AXIS_KEY_A2,
  AXIS_KEY_B2,
  AXIS_KEY_C2,
  AXIS_KEY_X3,
  AXIS_KEY_Y3,
  AXIS_KEY_Z3,
  AXIS_KEY_A3,
  AXIS_KEY_B3,
  AXIS_KEY_C3
  };

  typedef struct __packed CoordinateInformation {
  uint8_t axis;
  int32_t value;
} coordinate_info_t;

typedef struct __packed MovingCommand {
  uint8_t  axis_num;
  coordinate_info_t position[5];
  uint16_t feedrate;
} moving_command_t;

// This variable define in G0_G1.cpp
#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern feedRate_t fast_move_feedrate;
#endif

class MotionService {
  public:
    MotionService() {}

    void init();

    void pins_post_init();

    err_code_t pause_marlin(uint32_t timeout = 180 * 1000);
    err_code_t resume_marlin();

    // moving API
    void moveto_xy(float x, float y, float feedrate, bool blocked=true);
    void moveto_xyz(float x, float y, float z, float feedrate, bool blocked=true);
    void moveto_xyze(float x, float y, float z, float e, float feedrate, bool blocked=true) {}
    void moveto_x(float x, float feedrate, bool blocked=true);
    void moveto_y(float y, float feedrate, bool blocked=true);
    void moveto_z(float z, float feedrate, bool blocked=true);
    void moveto_a(float a, float feedrate, bool blocked=true) {}
    void moveto_b(float b, float feedrate, bool blocked=true) {}
    void moveto_e(float e, float feedrate, bool blocked=true);
    void moveto_e(float e, uint8_t extruder, float feedrate, bool blocked=true) {}
    void moveto(xyze_pos_t target, float feedrate, bool blocked=true);
    void synchronize_planner() {
      while (planner.busy()) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    bool is_axis_homed(ModuleLinearIndex axis) {
      switch (axis) {
      case MODULE_LINEAR_X1:
        return axis_was_homed(X_AXIS);

      case MODULE_LINEAR_Y1:
        return axis_was_homed(Y_AXIS);

      case MODULE_LINEAR_Z1:
        return axis_was_homed(Z_AXIS);

      case MODULE_LINEAR_Z2:
        return axis_was_homed(Z_AXIS);

      case MODULE_LINEAR_Y2:
        return axis_was_homed(Y_AXIS);

      case MODULE_LINEAR_X2:
        return axis_was_homed(X_AXIS);

      default:
        return false;
      }
    }
    bool endstop_status() { return endstops.global_enabled(); }
    void set_endstop(bool status) {
      endstops.enable_globally(status);
      soft_endstop._enabled = status;
    }
    void sync_feedrate_percentage_to_platform(int16_t percentage) { feedrate_percentage = percentage; }

    void req_quickstop(void);
    void normalstop(void);

    // home API
    bool is_all_axes_homed() {return all_axes_homed();}
    err_code_t home(bool block = true) { return run_gcode((char *)"G28 ", block); }
    err_code_t home_x(bool block = true) { return run_gcode((char *)"G28 X  ", block); }
    err_code_t home_y(bool block = true) { return run_gcode((char *)"G28 Y", block); }
    err_code_t home_z(bool block = true) { return run_gcode((char *)"G28 Z ", block); }
    void set_axis_to_homed(AxisEnum axis) { set_axis_homed(axis); }
    float get_home_offset(AxisEnum axis) { return home_offset[axis]; }

    // speed control API
    float get_feedrate(void);
    void set_feedrate(float);
    float get_travl_feedrate(void);
    void set_travl_feedrate(float);
    bool get_relative_mode(void);
    void set_relative_mode(bool);
    void set_feedrate_percentage(int16_t percentage) {}

    // position info API
    xyze_pos_t sm_current_position;
    xyze_pos_t sm_destination_position;
    void  update_position_from_platform() {
      sm_current_position = current_position;
    }
    float get_current_position(uint8_t axis) {
      update_position_from_platform();
      return current_position[axis];
    }
    void sync_plan_position_to_platform() {
      current_position = sm_current_position;
      sync_plan_position();
    }
    float get_max_position(uint8_t axis) {
      switch (axis) {
      case X_AXIS:
        return X_MAX_POS;

      case Y_AXIS:
        return Y_MAX_POS;

      case Z_AXIS:
        return Z_MAX_POS;

      case I_AXIS:
        return I_MAX_POS;

      case J_AXIS:
        return J_MAX_POS;

      default:
        return 0;
      }
    }
    float get_feedrate_percentage() { return feedrate_percentage; }

    // bed leveling API for internal app
    bool leveling_active() { return planner.leveling_active; }
    void disable_leveling() {set_bed_leveling_enabled(false);}
    void enable_leveling() {set_bed_leveling_enabled(true);}
    void set_bed_leveling_state(const bool state) { set_bed_leveling_enabled(state); }
    uint8_t get_leveling_grids();
    void get_leveling_first_point_position(float &x, float &y);
    void set_leveling_grids(uint8_t grids);
    void enable_z_probe() {endstops.enable_z_probe(true);}
    void disable_z_probe() {endstops.enable_z_probe(false);}
    float probe_at_point(float x, float y, ProbePtRaise raise_after=PROBE_PT_RAISE);
    void sync_leveling_limit_to_platform(float x_start, float x_end, float y_start, float y_end);
    void sync_z_values_to_platform();
    void sync_z_values_from_platform();
    void extrapolate_unprobed_points() {extrapolate_unprobed_bed_level();}
    void interpolate_virt_points() {refresh_bed_level();}
    void print_leveling_grid() { print_bilinear_leveling_grid();}
    void print_leveling_grid_virt() { print_bilinear_leveling_grid_virt();}
    bool get_leveling_state() { return leveling_is_valid(); }

    // extruder control API
    // uint8_t active_extruder() { return 0; }
    void update_active_extruder_to_platform(uint8_t e) { active_extruder = e; }

    // temperature API
    float current_hotend_temp(uint8_t heater_id = 0) { return 0.0; }
    int16_t target_hotend_temp(uint8_t heater_id = 0) { return 0.0; }
    float current_bed_temp(uint8_t area_id = 0);
    int16_t target_bed_temp(uint8_t area_id = 0);
    uint16_t get_bet_temp(void);
    bool set_bet_temp(uint16_t);

    // fdm API
    bool runout_state(uint8_t extruder = 0) { return false; }
    void sync_hotend_offset_to_platform(float x_offset, float y_offset, float z_offset);

    // settings control
    void reset_settings() {}
    void load_settings();
    void save_settings();

    err_code_t run_gcode(char *gcode, bool blocked = false, uint32_t blocked_timeout=180000);
    bool consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line);

    int8_t get_active_coordinate_system() { return gcode.active_coordinate_system; }
    bool is_original_position_offset() {
      bool result = true;
      LOOP_LINEAR_AXES(i) {
        if (position_shift[i] != gcode.coordinate_system[i]) {
          result = false;
        }
      }
      return result;
    }

    static void motion_background(void *p);
    static uint16_t hmi_cb_publish_coordinate_info(void *obj, uint8_t *buffer);
    static err_code_t hmi_cb_get_coordinate_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_active_coordinate_system(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_origin(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_move_absoluty(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_request_home(void *obj, sacp_hmi_message_t *msg);

    void show_coordiantes();

  private:
    MessageBufferHandle_t gcode_queue;
    xSemaphoreHandle marlin_signal;
    bool marlin_paused;
};


extern MotionService motion_svc;

#endif  // #ifndef SNAPMAKER_MOTION_SERVICE_H_
