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
#ifndef SNAPMAKER_MOTION_PLATFORM_SERVICE_H_
#define SNAPMAKER_MOTION_PLATFORM_SERVICE_H_

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
#include "../../../Marlin/src/core/serial.h"


// X Y Z I J K
#define AXIS_NUM  6
#define XYZ 3

#define SOFT_ENDSTOP_MIN (0)
#define SOFT_ENDSTOP_MAX (1)

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

#define MOTION_REQUEST_MAX    (8)
#define MOTION_REQ_GCODE_SIZE (80)
enum MotionRequestType {
  MQ_TYPE_DIRECT_ABSOLUTE,
  MQ_TYPE_DIRECT_RELATIVE,
  MQ_TYPE_GCODE,
  MQ_TYPE_HOME,
  MQ_TYPE_CHANGE_TOOL,
  MQ_TYPE_SYNC_PLAN_POSITION,

  MQ_TYPE_INVALID
};

enum MotionRequestState {
  MQ_STATE_IDLE,
  MQ_STATE_RECEIVED,
  MQ_STATE_PLANNED,
  MQ_STATE_MOVING,
  MQ_STATE_END,
};

typedef struct motion_request {
  MotionRequestType type;
  MotionRequestState current_state;
  MotionRequestState to_be_state;
  bool blocked;
  SemaphoreHandle_t ack;
  list_node node;

  union {
    struct {
      uint32_t index;
      bool compensate_z;
    } change_tool;
    struct {
    xyze_pos_t position;
    float      feedrate;
    } target;
    char gcode[MOTION_REQ_GCODE_SIZE];
  };
} motion_request_t;

class MotionPlatformService {
  public:
    bool homing_now;
    bool tool_changing;
    bool is_running_m600;
    uint32_t gcode_m600_line;

    MotionPlatformService() {}
    void init();
    void pins_post_init();

    // time API
    uint32_t get_millis() { return millis(); }

    // communication api
    void print_string_to_console(char *str) { print_to_console(str); }
    uint8_t get_console_protocol_type() { return SERIAL_IMPL.get_active_channel(); }

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
    void moveto(const xyze_pos_t &target, float feedrate, bool blocked=true);
    void synchronize_planner(void);

    bool is_axis_homed(ModuleLinearIndex axis);
    bool endstop_status() { return endstops.global_enabled(); }
    void set_endstop(bool status);

    void sync_feedrate_percentage_to_platform(int16_t percentage) { feedrate_percentage = percentage; }
    void sync_flowrate_percentage_to_platform(int16_t percentage, uint8_t e) { planner.set_flow(e, percentage); }
    int16_t get_flowrate_percentage(uint8_t e) { return planner.flow_percentage[e]; }

    void set_steps_per_unit(float steps_per_unit, uint8_t axis);
    float get_steps_per_unit(uint8_t axis);
    void set_e_axis_enable_on_state(uint8_t state) { E_ENABLE_ON = state; }

    // emergency_handle will call this API in ISR to stop motion platform
    uint32_t planner_clean_cnt;
    void req_emergency_stop();
    void req_quickstop(uint32_t clean_count=TEMP_TIMER_FREQUENCY);
    // void req_live_Z_offset_quickstop(void);
    bool planner_busy(void);
    bool is_moving();

    // This api use for wait planner quickstop
    err_code_t take_quickstop_sem(uint32_t wait_time);
    err_code_t give_quickstop_sem(void);
    void stepper_quickstop_sem_clear(void);
    void stepper_quickstop_finish(void);
    void stepper_quickstop_wait(void);
    void stepper_quickstop_cb(void);

    // home API
    bool is_all_axes_homed() {return all_axes_homed();}
    err_code_t home(bool block = true) { return run_gcode((char *)"G28", block); }
    err_code_t home_x(bool block = true) { return run_gcode((char *)"G28 X", block); }
    err_code_t home_y(bool block = true) { return run_gcode((char *)"G28 Y", block); }
    err_code_t home_z(bool block = true) { return run_gcode((char *)"G28 Z", block); }
    err_code_t home_b(bool block = true) { return run_gcode((char *)"G28 B", block); }
    void set_axis_to_homed(AxisEnum axis) { set_axis_homed(axis); }
    float get_home_offset(AxisEnum axis) { return home_offset[axis]; }
    void home_offset_init();
    void set_home_offset(float x, float y, float z, float i=0, float j=0);
    void set_laser_crosslight_offset(float ox, float oy);
    void get_laser_crosslight_offset(float &ox, float &oy);
    err_code_t check_cross_light_offset(float x_offset, float y_offset);

    // speed control API
    float get_feedrate(void);
    void set_feedrate(float);
    float get_travl_feedrate(void);
    void set_travl_feedrate(float);
    axis_bits_t get_relative_mode(void);
    void set_relative_mode(axis_bits_t);
    void set_feedrate_percentage(int16_t percentage) {}

    // position info API
    xyze_pos_t sm_current_position;
    xyze_pos_t sm_destination_position;
    void  update_position_from_stepper();
    void  update_position_from_platform();
    float get_current_position(uint8_t axis);
    void sync_plan_position_to_platform();
    float get_max_position(uint8_t axis);
    int16_t get_feedrate_percentage();
    xyz_pos_t get_position_shift();
    xyz_pos_t get_active_coordinate_system(int8_t active_id);
    void update_soft_endstops(uint8_t axis, uint8_t old_tool_index, uint8_t new_tool_index);
    void update_soft_endstops(uint8_t axis, uint8_t minmax, float val);
    void set_soft_endstops(uint8_t axis, uint8_t minmax, float val);
    float get_soft_endstop_min(uint8_t axis);
    float get_soft_endstop_max(uint8_t axis);
    uint32_t get_stepper_count(const AxisEnum axis);
    void set_stepper_count(const AxisEnum axis, uint32_t count_pos);

    // bed leveling API for internal app
    bool leveling_active() { return planner.leveling_active; }
    void disable_leveling() {set_bed_leveling_enabled(false);}
    void enable_leveling() {set_bed_leveling_enabled(true);}
    void set_bed_leveling_state(const bool state) { set_bed_leveling_enabled(state); }
    uint8_t get_leveling_grids();
    void get_leveling_first_point_position(float &x, float &y);
    void set_leveling_grids(uint8_t grids);
    void enable_x_probe() {endstops.enable_x_probe(true);}
    void disable_x_probe() {endstops.enable_x_probe(false);}
    void enable_y_probe() {endstops.enable_y_probe(true);}
    void disable_y_probe() {endstops.enable_y_probe(false);}
    void enable_z_probe() {endstops.enable_z_probe(true);}
    void disable_z_probe() {endstops.enable_z_probe(false);}
    float probe_at_point(float x, float y, ProbePtRaise raise_after=PROBE_PT_RAISE);
    float probe_x(float probe_position);
    float probe_y(float probe_position);
    void sync_leveling_limit_to_platform(float x_start, float x_end, float y_start, float y_end);
    void sync_z_values_to_platform(float compensation);
    void sync_manual_z_values_to_platform(float compensation);
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
    void set_hotend_temp(int16_t temp, int e=0);
    float get_hotend_temp(int e=0);
    int16_t get_bed_temp(int zone_index);
    void set_bed_temp(int16_t temp, int zone_index);
    bool bed_heatup_to_target(void);
    bool hotends_heatup_to_target(void);
    void set_hotend_maxtemp(uint8_t e, int16_t temp);
    void set_pid(uint8_t index, float value);

    // fdm API
    bool runout_state(uint8_t extruder = 0) { return false; }
    void sync_hotend_offset_to_platform(float x_offset, float y_offset, float z_offset);
    void enable_filament_runout(bool is_reset=false);
    void disable_filament_runout(bool is_reset=false);
    bool get_filament_runout(void);

    // settings control
    void reset_settings();
    void load_settings();
    void save_settings();

    err_code_t run_gcode(char *gcode, bool blocked = false, uint32_t blocked_timeout=180000);

    int8_t get_active_coordinate_system() { return gcode.active_coordinate_system; }
    bool is_original_position_offset();
    static void motion_background(void *p);
    static uint16_t hmi_cb_publish_coordinate_info(void *obj, uint8_t *buffer);
    static uint16_t hmi_cb_publish_feedrate(void *obj, uint8_t *buffer);
    static uint16_t hmi_cb_publish_feedrate_percentage(void *obj, uint8_t *buffer);
    static uint16_t hmi_cb_publish_job_print_time(void *obj, uint8_t *buffer);
    static err_code_t hmi_cb_get_coordinate_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_active_coordinate_system(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_origin(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_move_absoluty(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_request_home(void *obj, sacp_hmi_message_t *msg);
    // input shaper callbacks for HMI
    static err_code_t hmi_cb_set_inputshaper_frequency(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_get_inputshaper_frequency(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_inputshaper_switch(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_get_inputshaper_state(void *obj, sacp_hmi_message_t *msg);

    void show_coordiantes();
    void show_inputshaper_debug_info();
    void reset_inputshaper_debug_info();

    void reset_linear_drivers();

    void stop();
    void run();

    void do_quickstop();

    float get_motherboard_current_temp(uint8_t index);
    void abort_heating();

    void dispatch_motion_request();
    void run_motion_request(motion_request_t *mq);

    motion_request_t *malloc_motion_request(MotionRequestType target_type);
    err_code_t submit_motion_request(motion_request_t *mq, MotionRequestState sta = MQ_STATE_END);
    void wait_for_motion_request(motion_request_t *mq);

  private:
    void init_motion_request();
    void reset_motion_request();

  private:
    SemaphoreHandle_t quickstop_in_stepper_binary_sem;
    SemaphoreHandle_t quickstop_binary_sem;

    motion_request_t motion_request_cache[MOTION_REQUEST_MAX];
    SemaphoreHandle_t motion_request_lock;
    QueueHandle_t mq_list;
    bool quick_stop_mq;
};


extern MotionPlatformService motion_platform_svc;

#endif  // #ifndef SNAPMAKER_MOTION_SERVICE_H_
