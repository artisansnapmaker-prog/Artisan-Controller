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
#include "toolhead_cnc.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "src/core/millis_t.h"

// debug function: emergency stop
#include "src/feature/e_parser.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"
extern EmergencyParser emergency_parser;

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
  if (data[2]) {
    if (!(cnc.error_state & CNC_STALL_ERROR_MASK)) {
      // cnc stall
      LOG_I("cnc stall!\n");
      cnc.cnc_debug_emergency_stop();
    }
    cnc.error_state |= CNC_STALL_ERROR_MASK;
  }
  else {
    cnc.error_state &= (~CNC_STALL_ERROR_MASK);
  }
  cnc.keep_alive();
  cnc.online = true;
}

void ToolHeadCNC::lost_counter_routine(uint32_t time_out) {
  if (online) {
    if (ELAPSED(xTaskGetTickCount(),lost_counter + time_out)) {
      online = false;
    }
  }
}

err_code_t cnc_callback_routine(void *obj) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  cnc.lost_counter_routine();
  return E_SUCCESS;
}


err_code_t ToolHeadCNC::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  // TODO: insertion port of the detection module

  power = 0;
  rpm   = 0;
  output_sta = CNC_OUTPUT_OFF;
  ctr_mode  = CNC_CONSTANT_POWER_MODE;
  error_state = 0;

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
  keep_alive();

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

void ToolHeadCNC::set_power(uint8_t new_power) {
  if (new_power > CNC_POWER_MAX)
    new_power = CNC_POWER_MAX;

  power = new_power;
}


err_code_t ToolHeadCNC::set_output_power(uint8_t new_power) {
  set_power(new_power);
  return sync_cnc_output(power);
}


void ToolHeadCNC::turn_on() {
  sync_cnc_output(power);
};

void ToolHeadCNC::turn_off() {
  sync_cnc_output(0);
}

void ToolHeadCNC::report_cnc_status_info() {
  LOG_I("CNC rpm: %d, error: 0x%x\n",get_rpm(), get_error_state());
}


err_code_t ToolHeadCNC::sync_cnc_output(uint16_t value, CNCSpeedControlType type) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2];

  if (type == CNC_RPM_SET_SPEED) {
    LOG_E("The current module does not support setting rpm\n");
    return E_INVALID_CMD;
  }

  msg.id = get_message_id(MODULE_FUNC_SET_SPINDLE_SPEED);

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set CNC speed\n");
    return E_FAILURE;
  }

  if (value > CNC_POWER_MAX)
    value = CNC_POWER_MAX;

  buffer[0] = (uint8_t)value;

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set CNC out, ret: %u\n", ret);
    return ret;
  }

  if (value > 0)
    output_sta = CNC_OUTPUT_ON;
  else
    output_sta = CNC_OUTPUT_OFF;

  return E_SUCCESS;
}

void ToolHeadCNC::cnc_debug_emergency_stop() {
  OUT_WRITE(POWER_CTRL_8P, POWER_CTRL_OFF);
  emergency_parser.killed_by_M112 = true;
}
