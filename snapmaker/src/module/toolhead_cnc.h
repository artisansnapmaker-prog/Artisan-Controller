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
#ifndef SNAPMAKER_TOOLHEAD_CNC_H_
#define SNAPMAKER_TOOLHEAD_CNC_H_

#include "base.h"

#define CNC_POWER_MAX (100)
#define CNC_LOST_TIME_OUT  (3000)         

#define CNC_STALL_ERROR_MASK           (1 << 0)
#define CNC_H_PROTECT_ERROR_MASK       (1 << 1)
#define CNC_OVERCURRENT_ERROR_MASK     (1 << 2)
#define CNC_PCB_TEMP_ERROR_MASK        (1 << 3)
#define CNC_MOTOR_TEMP_ERROR_MASK      (1 << 4)
#define CNC_VOLTAGE_ERROR_MASK         (1 << 5)

enum CNCOutputStatus {
  CNC_OUTPUT_OFF = 0,
  CNC_OUTPUT_ON = 1,
  CNC_OUTPUT_INVALID
};

enum CNCSpeedControlType {
  CNC_PWM_SET_SPEED = 0,
  CNC_RPM_SET_SPEED,
};

enum CNCSpeedControlMode {
  CNC_CONSTANT_POWER_MODE = 0,
  CNC_CONSTANT_RPM_MODE,
};

class ToolHeadCNC: public ModuleBase {
  public:
		ToolHeadCNC(uint32_t mac, uint8_t key): ModuleBase(mac, key) {
      power     = 0;
      rpm       = 0;
    }

    virtual err_code_t pre_init();
    virtual err_code_t post_init();
    virtual err_code_t deinit();

    virtual bool check_online() { return online; }
    virtual void update_online(bool new_online) { online = new_online; }

    virtual uint8_t get_lost_counter() { return lost_counter; }
    virtual void set_lost_counter(uint8_t new_lost_counter) { lost_counter = new_lost_counter; }
    virtual void lost_counter_routine(uint32_t time_out=CNC_LOST_TIME_OUT);

    virtual uint16_t get_rpm() { return rpm; }
    virtual void set_rpm(uint16_t new_rpm) { rpm = new_rpm; }

    virtual uint16_t get_error_state() { return error_state; }
    virtual void set_error_state(uint16_t new_error_state) { error_state = new_error_state; }

    virtual CNCSpeedControlMode get_ctr_mode() { return (CNCSpeedControlMode)ctr_mode; }
    virtual void set_ctr_mode(CNCSpeedControlMode new_ctr_mode) { ctr_mode = new_ctr_mode; }

    virtual uint16_t get_target_rpm() { return target_rpm; }
    virtual void set_target_rpm(uint16_t new_target_rpm) { target_rpm = new_target_rpm; }

    virtual uint8_t get_power() { return power; }
    virtual void set_power(uint8_t new_power);

    virtual CNCOutputStatus get_output_sta() { return output_sta; }
    virtual void set_output_sta(CNCOutputStatus new_output_sta) { output_sta = new_output_sta; }

    virtual err_code_t set_output_power(uint8_t new_power);
    virtual err_code_t set_output_rpm(uint16_t new_rpm) { return E_INVALID_CMD; } 
    virtual err_code_t set_run_mode(CNCSpeedControlMode new_mode) { return E_INVALID_CMD; }

    virtual void turn_on();
    virtual void turn_off();

    virtual void keep_alive() { lost_counter = xTaskGetTickCount(); }
    virtual void report_cnc_status_info();

    virtual err_code_t cnc_debug_function(uint8_t cmd, uint32_t param) { return E_INVALID_CMD; }
    virtual void cnc_debug_emergency_stop();

    friend err_code_t cnc_callback_routine(void *obj);
    friend void cnc_callback_update_rpm(void *obj, uint8_t *data, uint8_t length);

  private:
    virtual err_code_t sync_cnc_output(uint16_t value, CNCSpeedControlType type=CNC_PWM_SET_SPEED);

  private:
    CNCOutputStatus output_sta;
    uint8_t power;
    uint8_t ctr_mode;
    uint16_t error_state;
    uint16_t rpm;
    uint16_t target_rpm;
    uint32_t lost_counter;
    
    bool  online = false;
};


#endif  // #ifndef TOOLHEAD_LASER_H_
