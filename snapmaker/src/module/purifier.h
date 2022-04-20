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
#ifndef SNAPMAKER_TOOLHEAD_PURIFIER_H_
#define SNAPMAKER_TOOLHEAD_PURIFIER_H_

#include "base.h"

#define PURIFIER_LOST_TIME_OUT           (3000) 
#define PURIFIER_SAMP_STATUS_INTERVAL    (500) 
#define PURIFIER_FAN_MAX_POWER           (100) 

#define SACP_PURIFIER_SUBSCRIBE_COMMANDID     0xa0

#define PURIFIER_ADDON_POWER_MASK             (1 << 0)
#define PURIFIER_EXTEND_POWER_OFF_MASK        (1 << 1)
#define PURIFIER_FAN_SPEED_TOO_LOW_MASK       (1 << 2)
#define PURIFIER_NO_FILTER_MASK               (1 << 3)
#define PURIFIER_ELEC_TOO_HIGH_MASK           (1 << 4)
#define PURIFIER_EMERGENCY_STOP_MASK          (1 << 5)

#define PURIFIER_START_WORK_OPEN_DEFAULT_MASK   (1 << PURIFIER_WORK_TOOL_HEAD_LASER)

#define PURIFIER_FDM_STOP_WORK_CLOSE_TIME_DELAY          (0)    // s
#define PURIFIER_LASER_STOP_WORK_CLOSE_TIME_DELAY        (300)   // s
#define PURIFIER_CNC_STOP_WORK_CLOSE_TIME_DELAY          (0)    // s

enum PurifierFanGear {
  PURIFIER_FAN_GEAR_0,  // off
  PURIFIER_FAN_GEAR_1,
  PURIFIER_FAN_GEAR_2,
  PURIFIER_FAN_GEAR_3,
};

enum PurifierReportInfoType {
  PURIFIER_INFO_LIFETIME,
  PURIFIER_INFO_ERR,
  PURIFIER_INFO_FAN_STA,
  PURIFIER_INFO_FAN_ELEC,
  PURIFIER_REPORT_POWER,
  PURIFIER_REPORT_STATUS,
  PURIFIER_INFO_ALL,
};

enum PurifierCtrlDevCmd {
  PURIFIER_SET_FAN_STA,
  PURIFIER_SET_FAN_GEARS,
  PURIFIER_SET_FAN_POWER,
  PURIFIER_SET_LIGHT,
};

enum PurifierLifetimeLevel {
  LIFETIME_LOW,
  LIFETIME_MEDIUM,
  LIFETIME_NORMAL,

  LIFETIME_INVALID
};

enum PurifierSacpRequestCommandId {
  SACP_CMD_ID_PURIFIER_GET_HEAD_INFO = 1,
  SACP_CMD_ID_PURIFIER_SET_FAN_GEARS,
  SACP_CMD_ID_PURIFIER_SET_FAN_ENABLE,
  SACP_CMD_ID_PURIFIER_SET_HEAD_START_WORK_STA,
  SACP_CMD_ID_PURIFIER_GET_HEAD_START_WORK_STA,
  SACP_CMD_ID_PURIFIER_SET_HEAD_STOP_WORK_STA,
  SACP_CMD_ID_PURIFIER_GET_HEAD_STOP_WORK_STA,

  // Fixed parameters, not modifiable
  SACP_CMD_ID_PURIFIER_END_INDEX,
  SACP_CMD_ID_PURIFIER_MAX_NUM = SACP_CMD_ID_PURIFIER_END_INDEX - 1,
};

enum PurifierWorkToolHeadIndex {
  PURIFIER_WORK_TOOL_HEAD_FDM, 
  PURIFIER_WORK_TOOL_HEAD_LASER,
  PURIFIER_WORK_TOOL_HEAD_CNC,

  PURIFIER_WORK_TOOL_HEAD_INVALID,
};

typedef struct {
  uint32_t start_work_purifier_open_mask;
  uint16_t fdm_stop_work_purifier_close_delay;
  uint16_t laser_stop_work_purifier_close_delay;
  uint16_t cnc_stop_work_purifier_close_delay;
}PurifierWorkSettings;

#pragma pack(1)

typedef struct {
  bool extend_power;
  bool fan_working_sta; 
  uint8_t fan_gear; 
  uint8_t life_time; 
  bool filter;
}PurifierStatus;

typedef struct {
  uint8_t key;
  uint8_t head_status; 
  PurifierStatus info;
}PurifierHeadInfo;

#pragma pack()

class Purifier: public ModuleBase {
  public:
    Purifier(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {}
  
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit();
    bool check_online() { return online; }

    err_code_t set_fan_power(uint8_t power);
    err_code_t set_fan_gear(uint8_t gear);
    err_code_t set_light_color(uint8_t red, uint8_t green, uint8_t blue);
    err_code_t set_fan_control(bool is_open, bool is_forced=false, uint16_t delay_s=0);
    void report_purifier_info(void);

    friend void start_work_notify_purifier_pro(void *obj, uint8_t reason);
    friend void stop_work_notify_purifier_pro(void *obj, uint8_t reason);
  
    friend err_code_t purifier_callback_routine(void *obj);
    friend void purifier_callback_update_info(void *obj, uint8_t *data, uint8_t length);
    
    // hmi
    friend err_code_t send_purifier_info_to_hmi(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_purifier_fan_gear(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_purifier_fan_ctrl(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_purifier_start_work_ctrl(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_get_purifier_start_work_ctrl(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_purifier_stop_work_ctrl(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_get_purifier_stop_work_ctrl(void *obj, sacp_hmi_message_t *msg);
    friend uint16_t hmi_subscribe_purifier_func(void *obj, uint8_t *buff);
  
  private:
    bool create_public_mutex_lock(); 
    bool public_mutex_lock(uint8_t retry=2, uint32_t timeout=100);
    void public_mutex_unlock();

    void purifier_offline_check(uint32_t time_out=PURIFIER_LOST_TIME_OUT);
    err_code_t get_purifier_info(PurifierReportInfoType report_type=PURIFIER_INFO_ALL, bool is_sysnc=false, uint32_t time_out=2000);
    err_code_t register_hmi_command_func(void *obj);
    err_code_t register_notify_handle_func(void *obj);
  
  private:
    bool online = false;
    bool fan_working_sta; 
    bool sysnc_get_info_flag = false; 
    uint8_t err;
    uint8_t fan_cur_out;
    uint8_t fan_gear;
    uint8_t lifetime;
    uint8_t sys_sta;
    uint8_t update_info_flag;
    uint16_t fan_speed;
    uint16_t fan_elec;
    uint16_t addon_power;
    uint16_t extend_power;
    uint32_t tick;
    uint32_t loop_next_time;
    uint32_t close_delay_tick;
    SemaphoreHandle_t public_mutex = NULL;
};

#endif // SNAPMAKER_TOOLHEAD_PURIFIER_H_
