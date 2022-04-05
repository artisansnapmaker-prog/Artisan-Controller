/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2022 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Controller2022-Marlin)
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
#ifndef SNAPMAKER_TOOLHEAD_CNC_200W_H_
#define SNAPMAKER_TOOLHEAD_CNC_200W_H_

#include "toolhead_cnc.h"

#define CNC_200W_DEFAULT_MAX_RPM   18000
#define CNC_200W_DEFAULT_MIN_RPM   8000

enum CNCConfigCmdType {
  CMD_SET_MOTOR_POWER = 0,
  CMD_SET_MOTOR_RPM,
  CMD_SET_MOTOR_RUN_MODE,
  CMD_SET_MOTOR_RUN_DIR,
  CMD_GET_MOTOR_PID_KP,
  CMD_GET_MOTOR_PID_KI,
  CMD_GET_MOTOR_PID_KD,
  CMD_SET_MOTOR_PID_KP,
  CMD_SET_MOTOR_PID_KI,
  CMD_SET_MOTOR_PID_KD,
};

class ToolHeadCNC200W: public ToolHeadCNC {
  public:
    ToolHeadCNC200W(uint32_t mac, uint8_t key, uint8_t sub_index): ToolHeadCNC(mac, key, sub_index) {}

    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit();   // TODO 

    err_code_t set_output_power(uint8_t new_power, bool is_update_power=true);
    err_code_t set_output_rpm(uint16_t new_rpm, bool is_update_rpm=true);
    err_code_t set_run_mode(CNCSpeedControlMode mode);

    void report_cnc_status_info();
    err_code_t debug_function(uint8_t cmd, uint32_t param);

    friend err_code_t hp_cnc_callback_routine(void *obj);
    friend void hp_cnc_callback_update_info(void *obj, uint8_t *data, uint8_t length);
    friend void hp_cnc_callback_update_sensor_info(void *obj, uint8_t *data, uint8_t length);
    friend void hp_cnc_callback_config_result(void *obj, uint8_t *data, uint8_t length);
  
  private:
    bool set_target_rpm(uint16_t new_rpm);
    bool is_support_rpm_mode() { return true; }
    bool is_support_change_ctr_mode() { return true; }
    virtual err_code_t sync_cnc_output(uint16_t power, CNCSpeedControlType type=CNC_PWM_SET_SPEED);
    bool get_enclosure_hw_verion(uint8_t *version);
    err_code_t set_cnc_run_dir(uint8_t dir); 
    err_code_t set_cnc_pid(uint8_t index, uint8_t mode, uint32_t param);

  private:
    float pcb_temp = 0;
    float motor_temp = 0;
    float motor_voltage = 0;
    uint16_t motor_current = 0;
};

#endif  // #ifndef SNAPMAKER_TOOLHEAD_CNC_200W_H_
