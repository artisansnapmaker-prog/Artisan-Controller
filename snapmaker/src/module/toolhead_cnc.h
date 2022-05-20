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
#define CNC_LIMIT_WORK_STATE_MASK      (CNC_OVERCURRENT_ERROR_MASK | CNC_PCB_TEMP_ERROR_MASK | \
                                        CNC_MOTOR_TEMP_ERROR_MASK | CNC_VOLTAGE_ERROR_MASK)

#define SACP_CNC_SUBSCRIBE_COMMANDID               0xa0

#define E_CNC_NO_SUPPORT        (PRIVATE_ERROR_BASE + 0)

enum CNCOutputStatus {
  CNC_OUTPUT_OFF = 0,
  CNC_OUTPUT_ON = 1,
  CNC_OUTPUT_OFF_ING = 2,
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

enum CNCCalibrationMode {
  CNC_KNIFE_PAIRING = 0,
  CNC_CHANGE_KNIFE,
  CNC_CUSTOM_KNIFE_PAIRING,
  CNC_MANUAL_KNIFE_PAIRING,
  CNC_CALIBRATION_IDLE
};

enum CNCSacpRequestCommandId {
  SACP_CMD_ID_CNC_GET_HEAD_INFO = 1,
  SACP_CMD_ID_CNC_SET_POWER,
  SACP_CMD_ID_CNC_SET_RPM,
  SACP_CMD_ID_CNC_SET_CTR_MODE,
  SACP_CMD_ID_CNC_SET_ENABLE,

  // Fixed parameters, not modifiable
  SACP_CMD_ID_CNC_END_INDEX,
  SACP_CMD_ID_CNC_MAX_NUM = SACP_CMD_ID_CNC_END_INDEX - 1,
};

enum CNCSacpCalibrationCommandId {
  SACP_CMD_ID_CNC_ENTER_CALIBRATE = 0,
  SACP_CMD_ID_CNC_EXIT_CALIBRATE,
  
  SACP_CMD_ID_CNC_CALIBRATE_NUM_MAX
};

#pragma pack(1)
typedef struct {
  uint8_t key;
  uint8_t head_status;
  bool head_active;    // cnc currently only has this false state
  uint8_t run_state; 
  uint8_t control_mode;
  uint8_t cur_power;   
  uint8_t target_power;   
  uint32_t cur_rpm; 
  uint32_t target_rpm; 
}CNCToolHeadInfo;

typedef struct {
  uint8_t key;
  uint8_t run_state; 
  uint8_t cur_power;   
  uint8_t target_power;   
  uint32_t cur_rpm; 
  uint32_t target_rpm;
  uint8_t control_mode;
}CNCSpeedState; 

#pragma pack()

class ToolHeadCNC: public ModuleBase {
  public:
		ToolHeadCNC(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {
      power     = 0;
      rpm       = 0;
    }

    virtual err_code_t pre_init();
    virtual err_code_t post_init();
    err_code_t deinit();
    err_code_t prepare_start(void);

    bool check_online() { return online; }
    uint16_t get_rpm() { return rpm; };
    err_code_t save_env(uint8_t *env_buf, uint32_t &len);
    err_code_t resume_env(uint8_t *env_buf, uint32_t &len);
    err_code_t standby(void);

    err_code_t set_feedrate_percentage(uint8_t *data, uint16_t length);
    uint16_t get_feedrate_percentage(uint8_t *buffer);

    virtual err_code_t set_output_power(uint8_t new_power, bool is_update_power=true);
    virtual err_code_t set_output_rpm(uint16_t new_rpm, bool is_update_rpm=true) { return E_INVALID_CMD; } 
    virtual err_code_t set_run_mode(CNCSpeedControlMode new_mode) { return E_INVALID_CMD; }

    virtual void report_cnc_status_info();
    virtual err_code_t start_spindle_self_test(void);
    virtual err_code_t debug_function(uint8_t cmd, uint32_t param) { return E_INVALID_CMD; }
    virtual void cnc_hmi_self_test_interface(uint8_t test_type, uint32_t param);

    friend err_code_t cnc_callback_routine(void *obj);
    friend void cnc_callback_update_rpm(void *obj, uint8_t *data, uint8_t length);

    // register handler functions for handling screen commands
    friend err_code_t send_cnc_head_info_to_hmi(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_power(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_rpm(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_ctr_mode(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_enable(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_enter_calibrate(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_cnc_exit_calibrate(void *obj, sacp_hmi_message_t *msg);
    friend uint16_t hmi_subscribe_cnc_func(void *obj, uint8_t *buff);

  protected:  
    bool create_public_mutex_lock(); 
    bool public_mutex_lock(uint8_t retry=2, uint32_t timeout=100);
    void public_mutex_unlock();
    void lost_counter_routine(uint32_t time_out=CNC_LOST_TIME_OUT);
    bool set_power(uint8_t new_power);
    bool set_calibrate_mode(CNCCalibrationMode mode);
    bool set_speed_feedrate(uint16_t feedrate);
    virtual bool set_target_rpm(uint16_t new_rpm) { return false; };
    virtual bool is_support_rpm_mode() { return false; }
    virtual bool is_support_change_ctr_mode() { return false; }
    virtual err_code_t register_hmi_command_func(void *obj);

  private:
    virtual err_code_t sync_cnc_output(uint16_t value, CNCSpeedControlType type=CNC_PWM_SET_SPEED);

  protected:
    CNCOutputStatus output_sta;
    CNCCalibrationMode calibrate_mode;
    uint8_t power;
    uint8_t real_power;
    uint8_t ctr_mode;
    uint16_t error_state;
    uint16_t rpm;
    uint16_t target_rpm;
    uint16_t record_error;
    uint32_t lost_counter;
    bool  online = false;
    int16_t feedrate_percentage;
    SemaphoreHandle_t public_mutex = NULL;
};
#endif  // #ifndef SNAPMAKER_TOOLHEAD_CNC_H_

