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

enum CNCConfigCmdType {
  CMD_SET_MOTOR_POWER = 0,
  CMD_SET_MOTOR_RPM,
  CMD_SET_MOTOR_RUN_MODE,
  CMD_SET_MOTOR_RUN_DIR,
  CMD_SET_MOTOR_PID_KP,
  CMD_SET_MOTOR_PID_KI,
  CMD_SET_MOTOR_PID_KD,
  CMD_GET_MOTOR_PID_VALUE,
};

class ToolHeadCNC200W: public ToolHeadCNC {
  public:
    ToolHeadCNC200W(uint32_t mac, uint8_t key): ToolHeadCNC(mac, key) {}
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t param_init();
    // err_code_t deinit();   // TODO 

    err_code_t set_output_rpm(uint16_t new_rpm);
    err_code_t set_run_mode(CNCSpeedControlMode mode);
    err_code_t send_cnc_cmd_config(uint8_t cmd_type, uint32_t param);
    // err_code_t set_output_power(uint8_t new_power);

    
    // void turn_on();    // TODO 
    // void turn_off();   // TODO 
    void report_cnc_status_info();

    virtual err_code_t cnc_debug_function(uint8_t cmd, uint32_t param);

    friend err_code_t hp_cnc_callback_routine(void *obj);
    friend void hp_cnc_callback_update_info(void *obj, uint8_t *data, uint8_t length);
    friend void hp_cnc_callback_update_sensor_info(void *obj, uint8_t *data, uint8_t length);
    friend void hp_cnc_callback_config_result(void *obj, uint8_t *data, uint8_t length);

  private:
    float pcb_temp = 0;
    float motor_temp = 0;
    float motor_voltage = 0;
    uint16_t motor_current = 0;
    uint16_t error_state_bak = 0;
    virtual err_code_t sync_cnc_output(uint16_t power, CNCSpeedControlType type=CNC_PWM_SET_SPEED);
};

#endif