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
#ifndef SNAPMAKER_ENCLOSURE_H_
#define SNAPMAKER_ENCLOSURE_H_

#include "base.h"

#define ENCLOSURE_DOOR_OPEN_STATUS       (1)
#define ENCLOSURE_DOOR_CLOSE_STATUS      (0)

#define ENCLOSURE_FAN_SPEED_DEFAULT      (255)
#define ENCLOSURE_LIGHT_POWER_DEFAULT    (255)

#define ENCLOSURE_OFFLINE_TIMEOUT        (3000)   //ms
#define ENCLOSURE_SAMP_STATUS_INTERVAL   (500)   //ms

#define ENCLOSURE_INVALID_DATA           0xFF
#define ENCLOSURE_INITIAL_STATE          (1 << 1)

#define ENCLOSURE_DOOR_STATUS_MASK              (1 << 0)
#define ENCLOSURE_LIGHT_LIMIT_STATUS_MASK       (1 << 1)
#define ENCLOSURE_HALL_1_STATUS_MASK            (1 << 2)
#define ENCLOSURE_HALL_2_STATUS_MASK            (1 << 3)

#define SACP_ENCLOSURE_SUBSCRIBE_COMMANDID      0xa0

// default laser enable enclosure detection
#define ENCLOSURE_CHECK_ENABLE_DEFAULT_MASK     (1 << ENCLOSURE_WORK_TYPE_LASER) 

enum EnclosureSacpRequestCommandId {
  SACP_CMD_ID_ENCLOSURE_GET_HEAD_INFO= 1,
  SACP_CMD_ID_ENCLOSURE_SET_LIGHT_LEVEL,
  SACP_CMD_ID_ENCLOSURE_ENABLE_CHECK,
  SACP_CMD_ID_ENCLOSURE_SET_FAN_SPEED,

  // Fixed parameters, not modifiable
  SACP_CMD_ID_ENCLOSURE_END_INDEX,
  SACP_CMD_ID_ENCLOSURE_MAX_NUM = SACP_CMD_ID_ENCLOSURE_END_INDEX - 1,
};

typedef struct {
  uint32_t enclosure_check_enable_mask;
}EnclosureSettings;

enum EnclosureWorkType {
  ENCLOSURE_WORK_TYPE_FDM, 
  ENCLOSURE_WORK_TYPE_LASER,
  ENCLOSURE_WORK_TYPE_CNC,
 
  ENCLOSURE_WORK_TYPE_LIMIT,
};


// #pragma pack(1)
// 
// typedef struct {
//   uint8_t key;
//   uint8_t head_status; 
//   uint8_t light_level; 
//   bool check_switch;
//   bool door_sta;   
//   uint8_t fan_speed;
// }EnclosureInfo;
// 
// #pragma pack()


class Enclosure: public ModuleBase {
  public:
		Enclosure(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {}

    virtual err_code_t pre_init();
    virtual err_code_t post_init();
    err_code_t deinit();

    void enclosure_test();

    bool check_online() { return online; }
    uint8_t get_enclosure_sta() { return enclosure_sta; }
    
    uint8_t get_door_check(void);
    uint32_t get_enclosure_check_mask(void);
    virtual err_code_t set_light_bar(uint8_t level);
    virtual err_code_t set_fan_speed(uint8_t speed);

    virtual void report_enclosure_status(); 
    virtual err_code_t get_enclosure_status();

    // virtual err_code_t enable_enclosure_check();
    // virtual err_code_t disable_enclosure_check();  

    virtual bool get_enclosure_check_switch_sta(void);

    virtual void enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param);  

    // enclosure process loop
    friend err_code_t enclosure_callback_routine(void *obj);
    friend void enclosure_callback_update_status(void *obj, uint8_t *data, uint8_t length);

    // register handler functions for handling screen commands
    friend err_code_t send_enclosure_info_to_hmi(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_light(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_fan(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_check(void *obj, sacp_hmi_message_t *msg);
    friend uint16_t hmi_subscribe_enclosure_func(void *obj, uint8_t *buff);

  protected:  
    bool create_public_mutex_lock(); 
    bool public_mutex_lock(uint8_t retry=3, uint32_t timeout=100);
    void public_mutex_unlock();
    virtual bool status_is_change(uint8_t cur_sta, uint8_t old_sta, uint8_t mask);
    virtual void enclosure_offline_check(uint32_t time_out=ENCLOSURE_OFFLINE_TIMEOUT);
    // dev_type: 0: light bar  1: fan
    virtual err_code_t set_enclosure_dev_func(uint8_t dev_type, uint8_t value, bool need_ack=false);
    virtual err_code_t register_hmi_command_func(void *obj);

  protected:
    bool  online = false;
    // bool  check_switch = true;
    uint8_t enclosure_sta;
    uint8_t light_limit = 0;
    uint8_t light_level = 0;
    uint8_t fan_speed = 0;
    uint32_t tick;
    uint32_t loop_next_time;
    SemaphoreHandle_t public_mutex = NULL;
};

#endif
