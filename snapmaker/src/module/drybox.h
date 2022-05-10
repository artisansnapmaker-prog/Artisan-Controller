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
#ifndef SNAPMAKER_DRYBOX_H_
#define SNAPMAKER_DRYBOX_H_

#include "base.h"

// #define ENABLE_DRYBOX_INTERRUPT_LOG

#define CHECK_ONLINE_TIMEOUT  5000

#define DRYBOX_SET_BIT(S, B) (S | (0x01 << B))
#define DRYBOX_CLR_BIT(S, B) (S & ~(0x01 << B))

typedef enum {
  DRYBOX_REQ_CMD_ID_GET_DRYBOX_INFO    = 1,
  DRYBOX_REQ_CMD_ID_SET_TEMP           = 2,
  DRYBOX_REQ_CMD_ID_SET_HEATING_TIME   = 3,
  DRYBOX_REQ_CMD_ID_HEATING_CTRL       = 4,

  DRYBOX_REQ_CMD_ID_SUM                = 4,      // Adding or deleting IDs requires changing this value
}drybox_req_cmd_id_e;

typedef enum {
  DRYBOX_SUBSCRIPT_CMD_ID_DRYBOX_STATE    = 0xa0,
}drybox_subscript_cmd_id_e;

typedef enum {
  DRYBOX_FAULT_OVER_TEMP,
  DRYBOX_FAULT_COVER_OPEN,
  DRYBOX_FAULT_HEATER_POWER,
  DRYBOX_FAULT_NO_EXTERNAL_POWER,
  DRYBOX_FAULT_DRYBOX_DISCONNECT,
}drybox_fault_state_e;

typedef enum {
  DRYBOX_STATE_IDLE,
  DRYBOX_STATE_HEATING_FREE,
  DRYBOX_STATE_HEATING_TIMING,
  DRYBOX_STATE_FAULT,
}drybox_state_e;

typedef struct {
  bool is_occupied;
  uint32_t peer;
  uint8_t ch;
}subscript_info_t;

#define HEATER_PREHEATING_TEMP    80

class DryBox: public ModuleBase {
  // public methods
  public:
    // construtor to do pre-init
    DryBox(uint32_t mac, uint8_t key, uint8_t sub_index):
    ModuleBase(mac, key, sub_index) {
      drybox_state = DRYBOX_STATE_IDLE;
      drybox_fault_state = 0;
      heating_state = 1;
      is_drybox_online = true;
      for (uint32_t i = 0; i < 2; i++) {
        subscript_info_array[i].is_occupied = false;
      }
    }

    bool check_online();
    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit() { return E_SUCCESS; }
    void update_temp_humidity(uint8_t *data);
    err_code_t set_mainctrl_type();
    err_code_t set_fan_speed(uint8_t speed, uint8_t delay_time);
    err_code_t set_temp(int16_t heater_temp, int16_t chamber_temp);
    err_code_t set_heating_time(uint32_t heating_time);
    err_code_t heating_ctrl(uint8_t state);
    err_code_t set_pid(float p, float i, float d);
    err_code_t get_pid();
    void report_pid(uint8_t *data);
    void update_heating_time_info(uint8_t type, uint32_t time);
    void update_heater_power_state(uint8_t state);
    void update_cover_state(uint8_t state);
    void update_drybox_state(drybox_state_e state);
    err_code_t heater_power_state_sync();
    err_code_t cover_state_sync();
    err_code_t drybox_state_sync();

    static void hmi_subscribe_drybox_status_notify_cb(void *obj, uint32_t peer, uint8_t ch, uint8_t type);
    static uint16_t hmi_subscript_callback_drybox_status(void *obj, uint8_t *buffer);
    void send_drybox_status_to_hmi();

    // hmi request callback
    static err_code_t hmi_req_callback_get_drybox_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_req_callback_set_temp(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_req_callback_set_heating_time(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_req_callback_heating_ctrl(void *obj, sacp_hmi_message_t *msg);

  // private methods
  private:


  // public properties
  public:
    bool is_drybox_online;

  // private properties
  private:
    uint16_t device_id;
    drybox_state_e drybox_state;
    volatile uint32_t drybox_fault_state;
    volatile uint8_t heating_state;
    volatile uint8_t cover_state;
    volatile uint8_t heater_power_state;
    volatile int16_t target_heater_temp;
    volatile int16_t target_chamber_temp;
    volatile uint16_t target_chamber_humidity;
    volatile int16_t current_heater_temp;
    volatile int16_t current_chamber_temp;
    volatile uint16_t current_chamber_humidity;
    volatile uint32_t target_heating_time;
    volatile uint32_t acc_heating_time;
    volatile uint32_t remaining_heating_time;
    uint32_t last_recv_time;
    subscript_info_t subscript_info_array[2];
};

#endif  // #ifndef SNAPMAKER_DRYBOX_H_

