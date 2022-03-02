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
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "toolhead_cnc_200w.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_CONFIG_SPINDLE, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_SPINDLE_RUN_INFO, MODULE_FUNC_PRIORITY_MEDIUM},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t ToolHeadCNC200W::param_init() {
  set_power(0);
  set_rpm(0);
  set_error_state(0);
  set_ctr_mode(CNC_CONSTANT_POWER_MODE);
  set_output_sta(CNC_OUTPUT_OFF);
  return E_SUCCESS;
}

err_code_t ToolHeadCNC200W::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  // TODO: insertion port of the detection module

  // param init
  param_init();
  return E_SUCCESS;
}

void hp_cnc_callback_update_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;

  // update cnc info
  if (length == 8) {
    cnc.set_rpm(data[0] << 8 | data[1]);
    cnc.set_error_state(data[2]);
    cnc.set_ctr_mode((CNCSpeedControlMode)data[5]);
    cnc.keep_alive();
    if (cnc.get_error_state()) {
      // new exception trigger
      if ((~cnc.error_state_bak) & cnc.get_error_state()) {  
        LOG_E("new exception trigger, error_state: 0x%x\n", data[2]);
        cnc.cnc_debug_emergency_stop();
      }
    }
    cnc.error_state_bak = data[2];
  }
}

void hp_cnc_callback_update_sensor_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  // update cnc sensor info
  if (length == 8) {
    cnc.motor_temp = (float)(data[0] << 8 | data[1]) / 10;
    cnc.pcb_temp = (float)(data[2] << 8 | data[3]) / 10;
    cnc.motor_current = data[4] << 8 | data[5];
    cnc.motor_voltage = (float)(data[6] << 8 | data[7]) / 100;
  }
}

void hp_cnc_callback_config_result(void *obj, uint8_t *data, uint8_t length) {
  // ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  // update cnc config result
  float pid_value = 0;
  switch(data[0]) {
    // TODO
    case 1:
    break;

    case 2:
    break;

    case 3:
      LOG_I("CNC dir set %s, current is %d\n", data[1] ? "success" : "fail",data[2]);
    break;

    case 4:
    case 5:
    case 6:
    break;

    case 7:
      pid_value = (float)(((data[2]) << 24) | ((data[3]) << 16) | ((data[4]) << 8) | (data[5])) / 1000;
      if (data[1] == 0) {
        LOG_I("get current Kp: %.3f\n", pid_value);
      }
      else if (data[1] == 1) {
        LOG_I("get current Ki: %.3f\n", pid_value);
      }
      else if (data[1] == 2) {
        LOG_I("get current Kd: %.3f\n", pid_value);
      }
    break;

    default:
    break;
  };
}

err_code_t hp_cnc_callback_routine(void *obj) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  cnc.lost_counter_routine();
  return E_SUCCESS;
}

err_code_t ToolHeadCNC200W::send_cnc_cmd_config(uint8_t cmd_type, uint32_t param) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[8];
  uint8_t i = 0;

  msg.id = get_message_id(MODULE_FUNC_CONFIG_SPINDLE);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set HP_CNC speed\n");
    return E_FAILURE;
  }

  buffer[i++] = (uint8_t)cmd_type;

  switch(cmd_type) {
    case CMD_SET_MOTOR_POWER:
      buffer[i++] = param > CNC_POWER_MAX ? CNC_POWER_MAX : (uint8_t)param;
    break;

    case CMD_SET_MOTOR_RPM:
      uint16_t tmp_rpm;
      tmp_rpm = param > 0xffff ? 0xffff : (uint16_t)param;
      buffer[i++] = (tmp_rpm >> 8) & 0xff;
      buffer[i++] = tmp_rpm & 0xff;
    break;

    case CMD_SET_MOTOR_RUN_MODE:
      buffer[i++] = (uint8_t)param;
    break;

    case CMD_SET_MOTOR_RUN_DIR:
      buffer[i++] = (uint8_t)param;
    break;

    case CMD_SET_MOTOR_PID_KP:
    case CMD_SET_MOTOR_PID_KI:
    case CMD_SET_MOTOR_PID_KD:
      buffer[i++] = (param >> 24) & 0xff;
      buffer[i++] = (param >> 16) & 0xff;
      buffer[i++] = (param >> 8) & 0xff;
      buffer[i++] = param & 0xff;
    break;

    case CMD_GET_MOTOR_PID_VALUE:
      break;

    default:
      LOG_E("no support cmd type\n");
      return E_FAILURE;
    break;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = i;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to send cnc cmd, ret: %u\n", ret);
  }
  return ret;
}

void ToolHeadCNC200W::report_cnc_status_info() {
  LOG_I("HP_CNC rpm: %d, error: 0x%x\n",get_rpm(), get_error_state());  
  LOG_I("HP_CNC M_I: %d, M_V: %.2f\n",motor_current, motor_voltage);
  LOG_I("HP_CNC M_TEMP: %.2f, PCB_TEMP: %.2f\n",motor_temp, pcb_temp);
  LOG_I("HP_CNC RUN MODE: %s\n",get_ctr_mode() ? "CNC_CONSTANT_RPM_MODE" : "CNC_CONSTANT_POWER_MODE");
}

err_code_t ToolHeadCNC200W::set_run_mode(CNCSpeedControlMode mode) {
  return send_cnc_cmd_config(CMD_SET_MOTOR_RUN_MODE, (CNCSpeedControlMode)mode);
}

err_code_t ToolHeadCNC200W::sync_cnc_output(uint16_t value, CNCSpeedControlType type) {
  err_code_t ret;
  smcan_message_t msg;
  CNCConfigCmdType cmd_type = CMD_SET_MOTOR_POWER;

  msg.id = get_message_id(MODULE_FUNC_CONFIG_SPINDLE);

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set HP_CNC speed\n");
    return E_FAILURE;
  }
  
  if (type == CNC_RPM_SET_SPEED)
    cmd_type = CMD_SET_MOTOR_RPM;

  ret = send_cnc_cmd_config(cmd_type, value);
  return ret;
}

err_code_t ToolHeadCNC200W::set_output_rpm(uint16_t new_rpm) {
  set_target_rpm(new_rpm);
  return sync_cnc_output(get_target_rpm(), CNC_RPM_SET_SPEED);
}

err_code_t ToolHeadCNC200W::cnc_debug_function(uint8_t cmd, uint32_t param) {
  return send_cnc_cmd_config(cmd, param);
}

err_code_t ToolHeadCNC200W::post_init() {
  uint16_t msg_id = get_message_id(MODULE_FUNC_REPORT_SPINDLE_RUN_INFO);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message MODULE_FUNC_REPORT_SPINDLE_RUN_INFO\n");
    return E_FAILURE;
  }
    
  // register callback to handle cnc info from module
  if (host_can_rou.register_callback(msg_id, (void *)this, hp_cnc_callback_update_info) != E_SUCCESS)
    return E_FAILURE;

  msg_id = get_message_id(MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO\n");
    return E_FAILURE;
  }

  // register callback to handle sensor info from module
  if (host_can_rou.register_callback(msg_id, (void *)this, hp_cnc_callback_update_sensor_info) != E_SUCCESS)
    return E_FAILURE;

  msg_id = get_message_id(MODULE_FUNC_CONFIG_SPINDLE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message MODULE_FUNC_CONFIG_SPINDLE\n");
    return E_FAILURE;
  }

  // register callback to handle sensor info from module
  if (host_can_rou.register_callback(msg_id, (void *)this, hp_cnc_callback_config_result) != E_SUCCESS)
    return E_FAILURE;

  smprinter.register_module(MODULE_DEVICE_ID_CNC_200W_2021, this);

  update_online(true);
  keep_alive();

  module_svc.register_routine((void *)this, hp_cnc_callback_routine);

  LOG_I("Hight power CNC ready!\n");
  return E_SUCCESS;
}