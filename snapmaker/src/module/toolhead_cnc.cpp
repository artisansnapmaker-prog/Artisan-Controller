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
#include "toolhead_cnc.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_SPINDLE_SPEED, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_GET_SPINDLE_SPEED, MODULE_FUNC_PRIORITY_LOW},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};


// message id callback to handle RPM update from module
void cnc_callback_update_rpm(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;

  cnc.rpm = (data[0]<<8 | data[1]);
  // LOG_I("rpm: %u", data[0]<<8 | data[1]);

  cnc.lost_counter = 0;
  cnc.online = true;
}


err_code_t cnc_callback_routine(void *obj) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;

  if (++cnc.lost_counter > CNC_LOST_MAX) {
    cnc.online = false;
  }
}


err_code_t ToolHeadCNC::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  power = 0;
  rpm   = 0;
  output_sta = CNC_OUTPUT_OFF;

  return E_SUCCESS;
}


err_code_t ToolHeadCNC::post_init() {
  uint16_t msg_id = get_message_id(MODULE_FUNC_GET_SPINDLE_SPEED);
  if (msg_id == MODULE_MESSAGE_ID_INVALID)
    return E_FAILURE;

  // register callback to handle RPM from module
  if (host_can_rou.register_callback(msg_id, (void *)this, cnc_callback_update_rpm) != E_SUCCESS)
    return E_FAILURE;

  smprinter.register_module(MODULE_DEVICE_ID_CNC_50W_2019, this);

  online = true;
  lost_counter = 0;

  module_svc.register_routine((void *)this, cnc_callback_routine);

  LOG_I("CNC ready!\n");
  return E_SUCCESS;
}


err_code_t ToolHeadCNC::deinit() {
  power = 0;
  rpm   = 0;
  output_sta = CNC_OUTPUT_OFF;
  online = false;

  return E_SUCCESS;
}


void ToolHeadCNC::set_output(uint8_t new_power) {
  if (new_power > CNC_POWER_MAX)
    new_power = CNC_POWER_MAX;

  power = new_power;

  sync_power(power);
}


void ToolHeadCNC::turn_on() {
  sync_power(power);
};


void ToolHeadCNC::turn_off() {
  sync_power(0);
}


err_code_t ToolHeadCNC::sync_power(uint8_t power) {
  err_code_t ret;
  smcan_message_t msg;;
  uint8_t buffer[2];

  msg.id = get_message_id(MODULE_FUNC_SET_SPINDLE_SPEED);

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set CNC speed\n");
    return E_FAILURE;
  }

  buffer[0] = power;

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set CNC out, ret: %u\n", ret);
    return ret;
  }

  if (power > 0)
    output_sta = CNC_OUTPUT_ON;
  else
    output_sta = CNC_OUTPUT_OFF;

  return E_SUCCESS;
}
