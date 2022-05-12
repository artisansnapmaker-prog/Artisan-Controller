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
#ifndef SNAPMAKER_TOOLHEAD_FDM_H_
#define SNAPMAKER_TOOLHEAD_FDM_H_

#include "base.h"

// #define USE_FDM_INTERRUPT_LOG

#define EXTRUDERS 2
#define EXTRUDER0_SWITCH_POSITION 0
#define EXTRUDER1_SWITCH_POSITION 410
#define GRID_MAX_NUM 11

#define SINGLE_EXTRUDER_SOFT_ENDSTOP_MIN_X      0
#define SINGLE_EXTRUDER_SOFT_ENDSTOP_MAX_X      400
#define DUAL_EXTRUDER_LEFT_SOFT_ENDSTOP_MAX_X   410
#define DUAL_EXTRUDER_LEFT_SOFT_ENDSTOP_MIN_X   0
#define DUAL_EXTRUDER_RIGHT_SOFT_ENDSTOP_MAX_X  410
#define DUAL_EXTRUDER_RIGHT_SOFT_ENDSTOP_MIN_X  0

#define DELAY_TURNOFF_TIME_MS      (5*60*1000)
#define DUAL_EXTRUDER_SAFE_SPACE_MIN_X          35
#define DUAL_EXTRUDER_SAFE_SPACE_MAX_X          35
#define DUAL_EXTRUDER_SAFE_SPACE_MIN_Y          2
#define DUAL_EXTRUDER_SAFE_SPACE_MAX_Y          2
#define DUAL_EXTRUDER_SAFE_SPACE_MAX_Z          6
#define TOOL_CHANGE_RAISE_SPACE                 1

#define DEFAULT_HOTEND_OFFSET_X                 24
#define DEFAULT_HOTEND_OFFSET_Y                 0
#define DEFAULT_HOTEND_OFFSET_Z                 -1.5
#define BIAS_HOTEND_OFFSET_X                    1.2
#define BIAS_HOTEND_OFFSET_Y                    1.2
#define BIAS_HOTEND_OFFSET_Z                    1.2

#define CHECK_ONLINE_TIMEOUT  5000

#define SINGLE_EXTRUDER_STEPS_PER_UNIT_DEFAULT  212.21
#define DUAL_EXTRUDER_STEPS_PER_UNIT_DEFAULT    142


/****************************************************************************************
reference links: https://snapmaker2.atlassian.net/wiki/spaces/SNAP/pages/1984987369/FDM
****************************************************************************************/
typedef enum {
  FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO     = 1,
  FDM_REQ_CMD_ID_SET_HOTEND_TEMP       = 2,
  FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL  = 4,
  FDM_REQ_CMD_ID_SWITCH_EXTRUDER       = 5,
  FDM_REQ_CMD_ID_SET_FAN_SPEED         = 6,
  FDM_REQ_CMD_ID_SET_HOTEND_OFFSET     = 7,
  FDM_REQ_CMD_ID_GET_HOTEND_OFFSET     = 8,
  FDM_REQ_CMD_ID_EXTRUDER_MOTION       = 9,
  FDM_REQ_CMD_ID_CHANGE_NOZZLE_CTRL    = 10,

  FDM_REQ_CMD_ID_SUM                   = 9,      // Adding or deleting IDs requires changing this value
}fdm_req_cmd_id_e;

typedef enum {
  FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO = 0xa0,
}fdm_subscript_cmd_id_e;

typedef enum {
  SINGLE_EXTRUDER_MODULE_FAN       = 0,
  SINGLE_EXTRUDER_NOZZLE_FAN       = 1,
  DUAL_EXTRUDER_LEFT_MODULE_FAN    = 0,
  DUAL_EXTRUDER_RIGHT_MODULE_FAN   = 1,
  DUAL_EXTRUDER_NOZZLE_FAN         = 2,
}fan_e;
typedef enum {
  PROBE_SENSOR_PROXIMITY_SWITCH,
  PROBE_SENSOR_LEFT_OPTOCOUPLER,
  PROBE_SENSOR_RIGHT_OPTOCOUPLER,
  PROBE_SENSOR_LEFT_CONDUCTIVE,
  PROBE_SENSOR_RIGHT_CONDUCTIVE,
  PROBE_SENSOR_INVALID,
}probe_sensor_t;

typedef struct {
  uint8_t model;
  float diameter;
}hotend_type_info_t;

typedef struct {
  float single_extruder_steps_per_unit;
  float dual_extruder_steps_per_unit[EXTRUDERS];
}fdm_settings_t;

#define HOTEND_INFO_MAX 22
const hotend_type_info_t hotend_info[HOTEND_INFO_MAX] = {{.model = 2, .diameter = 0.4}, \
                                                         {.model = 1, .diameter = 0.2}, \
                                                         {.model = 1, .diameter = 0.6}, \
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0, .diameter = 0.4}, \
                                                         {.model = 0, .diameter = 0.8}, \
                                                        };

typedef struct {
  int16_t current;
  int16_t target;
}hotend_temp_t;

typedef enum {
    EXTRUDER_STATUS_CHECK,
    EXTRUDER_STATUS_IDLE,
}extruder_status_e;

typedef enum {
  FDM_FAULT_EXTRUDER_STATE,
  FDM_FAULT_NOZZLE_IDENTIFY,
  FDM_FAULT_NOZZLE_TEMP,
  FDM_FAULT_FILAMENT,
}fdm_fault_e;

typedef struct {
  uint8_t active_extruder;
  int16_t feedrate_percentage[EXTRUDERS];
  int16_t flowrate_percentage[EXTRUDERS];
  float live_z_offset[EXTRUDERS];
  float live_z_offset_changed;
  uint8_t fan_speed[3];
  int16_t target_temp[EXTRUDERS];
} __attribute__((packed)) fdm_recovery_data_t;

class ToolHeadFDM: public ModuleBase {
  // public methods
  public:
    // construtor to do pre-init
    ToolHeadFDM(uint8_t extruder, uint32_t mac, uint8_t key, uint8_t sub_index):
    ModuleBase(mac, key, sub_index) {
      fdm_state = 0;
      for (int i = 0; i < EXTRUDERS; i++) {
        hotend_type[i] = 0xff;
      }
      for (int i = 0; i < EXTRUDERS; i++) {
        hotend_temp[i].current = 0;
        hotend_temp[i].target  = 0;
      }
      probe_state    = 0;
      probe_sensor   = PROBE_SENSOR_PROXIMITY_SWITCH;
      extruder_info  = 0;
      active_extruder = 0;
      target_extruder = 0;
      hotend_type_initialized = false;
      memset(hotend_offset, 0, sizeof(hotend_offset));
      last_recv_time = 0;
      is_fdm_online = true;
    }

    bool check_online();
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t single_extruder_post_init();
    err_code_t dual_extruder_post_init();
    err_code_t deinit() { return E_SUCCESS; }
    err_code_t save_env(uint8_t *env_buf, uint32_t &len);
    err_code_t recover_env(uint8_t *env_buf, uint32_t &len);
    err_code_t resume_env(uint8_t *env_buf, uint32_t &len);
    err_code_t standby(void);
    void prepare_to_start_a_new_print_job(void);
    bool prepare_start(void);
    err_code_t set_feedrate_percentage(uint8_t *data, uint16_t length);
    uint16_t get_feedrate_percentage(uint8_t *buffer);
    err_code_t factory_reset();

    err_code_t probe_state_sync();
    err_code_t hotend_type_sync();
    err_code_t filament_state_sync();
    err_code_t hotend_offset_sync();
    err_code_t z_compensation_sync();
    err_code_t hotend_pid_sync();
    void set_probe_state(uint8_t state[]);
    void set_probe_state(probe_sensor_t sensor, uint8_t state);
    void report_pid(uint8_t *data);
    void set_hotend_type(uint8_t *data);
    void report_extruder_info(uint8_t *data);
    uint8_t get_hotend_type(uint8_t e);
    float get_hotend_diameter(uint8_t e);
    void set_probe_sensor(probe_sensor_t sensor);
    bool get_probe_state();
    bool get_probe_state(probe_sensor_t sensor);
    err_code_t set_pid(float p, float i, float d);
    void update_hotend_temp(uint8_t *data);
    err_code_t set_hotend_temp(uint16_t temp, uint8_t e);
    float get_hotend_temp(uint8_t e);
    float get_hotend_target_temp(uint8_t e);
    err_code_t set_fan_speed(uint8_t fan_index, uint16_t speed, uint8_t delay_time=0);
    uint8_t get_fan_speed(uint8_t fan_index);
    void update_filament_state(uint8_t *data);
    uint8_t get_filament_state(uint8_t e);
    uint8_t get_filament_state();
    uint8_t get_filament_detection_state(uint8_t e);
    uint32_t get_fdm_state();
    void clear_fdm_state(fdm_fault_e state);
    void get_fdm_state(fdm_fault_e state);
    uint8_t get_extruder_status(uint8_t e);
    err_code_t extruder_status_check_ctrl(extruder_status_e status);
    err_code_t tool_change(uint8_t new_tool, bool z_compensation=true);
    err_code_t switch_extruder(uint8_t e);
    void switch_extruder_without_move(uint8_t e);
    err_code_t get_hotend_offset(float &x_offset, float &y_offset, float &z_offset);
    err_code_t set_hotend_offset(float offset, uint8_t axis);
    void set_hotend_offset_z(float offset) { hotend_offset[2][1] = offset; }
    uint8_t get_extruders_count();
    err_code_t set_extruders_feedrate_percentage(int16_t percentage, uint8_t e);
    int16_t get_extruders_feedrate_percentage(uint8_t e);
    err_code_t set_extruders_flowrate_percentage(int16_t percentage, uint8_t e);
    int16_t get_extruders_flowrate_percentage(uint8_t e);
    err_code_t filament_detect_ctrl(uint8_t state, uint8_t e);
    uint8_t get_active_extruder();
    err_code_t save_hotend_offset_to_module(float offset, uint8_t axis);
    err_code_t save_z_compensation_to_module(float *compensation);
    float *get_hotend_pid(uint8_t e) { return pid; }
    void fdm_exception_trigger(fdm_fault_e fault);
    void fdm_exception_clear(fdm_fault_e fault);
    void show_fdm_info();
    void delay_turnoff_heating_process();
    void dual_extruder_process_after_z_homed();
    void report_hotend_offset();
    void report_nozzle_type();
    void set_axis_steps_per_unit(float value);
    void report_steps_per_unit();

  // private methods
  private:


  // public properties
  public:
    float hotend_offset[3][EXTRUDERS];
    bool is_fdm_online;

  // private properties
  private:
    uint32_t fdm_state;
    uint8_t probe_state;
    probe_sensor_t probe_sensor;
    uint8_t extruder_info;
    uint8_t hotend_type[EXTRUDERS];
    hotend_temp_t hotend_temp[EXTRUDERS];
    uint8_t filament_state;
    uint8_t active_extruder;
    uint8_t target_extruder;
    probe_sensor_t active_probe_sensor;
    uint8_t filament_detect_mask;
    uint8_t extruder_status[EXTRUDERS];
    float hotend_diameter[EXTRUDERS];
    uint8_t fan_speed[3];
    int16_t extruders_feedrate_percentage[EXTRUDERS];
    int16_t extruders_flowrate_percentage[EXTRUDERS];
    uint8_t filament_detect_state[EXTRUDERS];
    float pid[3];
    bool hotend_type_initialized;
    uint32_t turnoff_heating_time_elapsed;
    uint32_t last_recv_time;
    float single_extruder_steps_per_unit;
    float dual_extruder_steps_per_unit[EXTRUDERS];
};

#endif  // #ifndef SNAPMAKER_TOOLHEAD_FDM_H_

