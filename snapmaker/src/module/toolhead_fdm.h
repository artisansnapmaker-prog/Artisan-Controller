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

#define EXTRUDERS 2
#define TOOL_CHANGE_SAFE_SPACE 30
#define EXTRUDER0_SWITCH_POSITION 0
#define EXTRUDER1_SWITCH_POSITION 410

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

typedef enum {
  HOTEND_TYPE_0,
  HOTEND_TYPE_1,
  HOTEND_TYPE_2,
  HOTEND_TYPE_3,
  HOTEND_TYPE_4,
  HOTEND_TYPE_5,
  HOTEND_TYPE_6,
  HOTEND_TYPE_7,
  HOTEND_TYPE_8,
  HOTEND_TYPE_9,
  HOTEND_TYPE_10,

  HOTEND_TYPE_IDLE,
  HOTEND_TYPE_INVALID = 0xff,
}hotend_type_t;

typedef struct {
  int16_t current;
  int16_t target;
}hotend_temp_t;

typedef enum {
    EXTRUDER_STATUS_CHECK,
    EXTRUDER_STATUS_IDLE,
}extruder_status_e;

class ToolHeadFDM: public ModuleBase {
  // public methods
  public:
    // construtor to do pre-init
    ToolHeadFDM(uint32_t mac, uint8_t key, uint8_t extruder):
    ModuleBase(mac, key) {
      for (int i = 0; i < EXTRUDERS; i++) {
        hotend_type_[i] = HOTEND_TYPE_IDLE;
      }
      for (int i = 0; i < EXTRUDERS; i++) {
        hotend_temp_[i].current = 0;
        hotend_temp_[i].target  = 0;
      }
      probe_state_    = 0;
      probe_sensor_   = PROBE_SENSOR_PROXIMITY_SWITCH;
      extruder_info_  = 0;
      active_extruder_ = 0;
    }

    bool check_online() { return false; }
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }

    err_code_t probe_state_sync();
    err_code_t hotend_type_sync();
    err_code_t filament_state_sync();
    void set_probe_state(uint8_t state[]);
    void set_hotend_type(uint8_t *data);
    uint8_t get_hotend_type(uint8_t e);
    void set_probe_sensor(probe_sensor_t sensor);
    bool get_probe_state();
    bool get_probe_state(probe_sensor_t sensor);
    void update_hotend_temp(uint8_t *data);
    err_code_t set_hotend_temp(uint16_t temp, uint8_t e);
    float get_hotend_temp(uint8_t e);
    err_code_t set_fan_speed(uint8_t fan_index, uint16_t speed, uint8_t delay_time=0);
    void update_filament_state(uint8_t *data);
    uint8_t get_filament_state(uint8_t e);
    uint8_t get_filament_state();
    void extruder_status_check_ctrl(extruder_status_e status);
    err_code_t tool_change(uint8_t new_tool);
    err_code_t switch_extruder(uint8_t e);
    void switch_extruder_without_move(uint8_t e);

  // private methods
  private:
    uint8_t probe_state_;
    probe_sensor_t probe_sensor_;
    uint8_t extruder_info_;
    hotend_type_t hotend_type_[EXTRUDERS];
    hotend_temp_t hotend_temp_[EXTRUDERS];
    uint8_t filament_state_;
    uint8_t active_extruder_;
    probe_sensor_t active_probe_sensor_;
    float hotend_offset_[3][EXTRUDERS];

  // public properties
  public:


  // private properties
  private:
    uint16_t device_id_;

};

#endif  // #ifndef SNAPMAKER_TOOLHEAD_FDM_H_

