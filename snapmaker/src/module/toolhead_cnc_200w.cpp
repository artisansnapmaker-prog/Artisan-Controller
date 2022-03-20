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
#include "toolhead_cnc_200w.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "src/HAL/HAL.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_SPINDLE_SPEED, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_3DP_PID, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_GET_HW_VERSION, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_SPINDLE_RPM, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_MOTOR_CTR_MODE, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_MOTOR_RUN_DIRECTION, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_SPINDLE_RUN_INFO, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO, MODULE_FUNC_PRIORITY_MEDIUM},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t ToolHeadCNC200W::pre_init() {
  LOG_I("HP_CNC pre_init in\n");
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  // TODO: insertion port of the detection module

  // create resource lock
  if (!create_public_mutex_lock())
    return E_FAILURE;

  // param init
  if (public_mutex_lock()) {
    power = 0;
    rpm = 0;
    target_rpm = 0;
    error_state = 0;
    ctr_mode = CNC_CONSTANT_POWER_MODE;
    output_sta = CNC_OUTPUT_OFF;
    calibrate_mode = CNC_CALIBRATION_IDLE;
    online = false;
    set_status(MODULE_STATUS_INIT);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] HP_CNC take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  LOG_I("HP_CNC pre_init out\n");
  return E_SUCCESS;
}

// TODO 
err_code_t ToolHeadCNC200W::deinit(){
  return E_SUCCESS;
}

void hp_cnc_callback_update_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  uint8_t error_trigger = 0;
  if (!obj || !cnc.online)
    return;
  // update cnc info
  if (length == 8) {
    if (cnc.public_mutex_lock()) {
      if (!cnc.online) {
        cnc.public_mutex_unlock();
        return;
      }
      cnc.rpm = data[0] << 8 | data[1];
      // cnc.error_state = data[2];
      cnc.output_sta = (CNCOutputStatus)data[3];
      cnc.ctr_mode = (CNCSpeedControlMode)data[5];
      cnc.real_power = data[7];
      cnc.lost_counter = xTaskGetTickCount();
      if (data[2]) {
        if ((~cnc.error_state) & data[2]) { 
          error_trigger = 1; 
        }
      }
      // TODO: Modified state better after the event has been successfully 
      // processed, subsequent optimisation.
      cnc.error_state = data[2];
      cnc.public_mutex_unlock();
    }
    
    if (error_trigger) {
        LOG_E("new exception trigger, error_state: 0x%x\n", cnc.error_state);
        cnc.debug_emergency_stop();
    }
  }
}

void hp_cnc_callback_update_sensor_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  if (!obj || !cnc.online)
    return;
  // update cnc sensor info
  if (length == 8) {
    if (cnc.public_mutex_lock()) {
      if (!cnc.online) {
        cnc.public_mutex_unlock();
        return;
      }
      cnc.motor_temp = (float)(data[0] << 8 | data[1]) / 10;
      cnc.pcb_temp = (float)(data[2] << 8 | data[3]) / 10;
      cnc.motor_current = data[4] << 8 | data[5];
      cnc.motor_voltage = (float)(data[6] << 8 | data[7]) / 100;
      cnc.public_mutex_unlock();
    }
  }
}

err_code_t hp_cnc_callback_routine(void *obj) {
  ToolHeadCNC200W &cnc = *(ToolHeadCNC200W *)obj;
  if (obj){
    cnc.lost_counter_routine();
  }
  return E_SUCCESS;
}

bool ToolHeadCNC200W::get_enclosure_hw_verion(uint8_t *version) {
  smcan_message_t msg;
  bool ret = false;
  err_code_t result = E_FAILURE;
  uint8_t out_buf[8] = {0};
  uint8_t out_len = sizeof(out_buf);
  msg.id = get_message_id(MODULE_FUNC_GET_HW_VERSION);
  if (msg.id != MODULE_MESSAGE_ID_INVALID) {
    msg.ch     = get_channel();
    msg.data   = NULL;
    msg.length = 0;
  }
  result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
  if (result == E_SUCCESS) {
    *version = out_buf[0];
    ret = true;
  }
  return ret;
}

void ToolHeadCNC200W::report_cnc_status_info() {
  LOG_I("HP_CNC rpm: %d, error: 0x%x\n",rpm, error_state);  
  LOG_I("HP_CNC M_I: %d, M_V: %.2f\n",motor_current, motor_voltage);
  LOG_I("HP_CNC M_TEMP: %.2f, PCB_TEMP: %.2f\n",motor_temp, pcb_temp);
  LOG_I("HP_CNC ctr mode: %s\n",ctr_mode ? "CNC_CONSTANT_RPM_MODE" : "CNC_CONSTANT_POWER_MODE");
  LOG_I("HP_CNC run status: %s\n",output_sta == 0 ? "STOP" :  output_sta == 1 ? "RUN" : "STOPING");
  LOG_I("HP_CNC cur_power: %d target_power: %d\n",  real_power, power);
  LOG_I("HP_CNC cur_rpm: %d target_rpm: %d\n",  rpm, target_rpm);
  LOG_I("HP_CNC calibration mode: %d\n",calibrate_mode);
  LOG_I("HP_CNC mode status: %d\n",get_status());
}

err_code_t ToolHeadCNC200W::set_run_mode(CNCSpeedControlMode mode) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[2] = {0};
  uint8_t out_buf[8] = {0};
  uint8_t out_len = sizeof(out_buf);

  msg.id = get_message_id(MODULE_FUNC_SET_MOTOR_CTR_MODE);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }
  buffer[0] = (uint8_t)mode;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
  if (result == E_SUCCESS) {
    if (!out_buf[0]) {
      result = E_FAILURE;
      LOG_E("[%s] set fail\n",__FUNCTION__);
    }
  }
  else {
    LOG_E("[%s] send msg fail result: %d\n",__FUNCTION__, result);
  }
  return result;
}

err_code_t ToolHeadCNC200W::set_cnc_run_dir(uint8_t dir) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[2] = {0};
  uint8_t out_buf[8] = {0};
  uint8_t out_len = sizeof(out_buf);

  msg.id = get_message_id(MODULE_FUNC_SET_MOTOR_RUN_DIRECTION);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }
  buffer[0] = (uint8_t)dir;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
  if (result == E_SUCCESS) {
    if (!out_buf[0]) {
      result = E_FAILURE;
      LOG_E("[%s] set fail\n",__FUNCTION__);
    }
  }
  else {
    LOG_E("[%s] send msg fail result: %d\n", __FUNCTION__,result);
  }
  return result;
}

err_code_t ToolHeadCNC200W::set_cnc_pid(uint8_t index, uint8_t mode, uint32_t param) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[8] = {0};
  uint8_t out_buf[8] = {0};
  uint8_t out_len = sizeof(out_buf);  
  uint8_t i = 0;

  msg.id = get_message_id(MODULE_FUNC_SET_3DP_PID);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (mode)
    buffer[i++] = 0xfe;   // set
  else
    buffer[i++] = 0xff;   // get

  switch(index) {
    case 0:
    case 1:
    case 2:
    case 3:
      buffer[i++] = index;
      buffer[i++] = (param >> 24) & 0xff;
      buffer[i++] = (param >> 16) & 0xff;
      buffer[i++] = (param >> 8) & 0xff;
      buffer[i++] = param & 0xff;
    break;

    default:
      LOG_E("[%s] no support index\n",__FUNCTION__);
      return E_FAILURE;
    break;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = i;
  if (!mode) {
    result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
    if (result == E_SUCCESS) {
      float pid_value = (float)(((out_buf[1]) << 24) | ((out_buf[2]) << 16) | \
          ((out_buf[3]) << 8) | (out_buf[4])) / 1000;
      LOG_I("get current %s: %.3f\n", out_buf[0] == 0 ? "Kp" : out_buf[0] == 1 ? "Ki" : "Kd", pid_value);
    }
  }
  else 
    result = host_can_rou.send(&msg);

  if (result) {
    LOG_E("[%s] send msg fail result: %d\n", __FUNCTION__,result);
  }
  return result;
}

err_code_t ToolHeadCNC200W::sync_cnc_output(uint16_t value, CNCSpeedControlType type) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[2] = {0};
  uint8_t out_buf[8] = {0};
  uint8_t out_len = sizeof(out_buf);
  uint8_t i = 0;
  if (type == CNC_PWM_SET_SPEED)
    msg.id = get_message_id(MODULE_FUNC_SET_SPINDLE_SPEED);
  else 
    msg.id = get_message_id(MODULE_FUNC_SET_SPINDLE_RPM);

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message to set HP_CNC speed\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (type == CNC_PWM_SET_SPEED) {
    value = value > CNC_POWER_MAX ? CNC_POWER_MAX : value; 
    buffer[i++] = (uint8_t)value;
  }
  else {
    value = value > CNC_200W_DEFAULT_MAX_RPM ? CNC_200W_DEFAULT_MAX_RPM : value; 
    buffer[i++] = (value >> 8) & 0xff;
    buffer[i++] = value & 0xff;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = i;
  result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
  if (result) 
    LOG_E("[%s] send fail result: %d\n",__FUNCTION__, result);
  return result;
}

bool ToolHeadCNC200W::set_target_rpm(uint16_t new_rpm) {
  bool ret = false;
  if (public_mutex_lock()) {
    if (new_rpm > CNC_200W_DEFAULT_MAX_RPM)
      new_rpm = CNC_200W_DEFAULT_MAX_RPM;
    target_rpm = new_rpm;
    public_mutex_unlock();
    ret = true;
  }
  else 
    LOG_E("[%s] cnc take public_mutex_lock fail\n", __FUNCTION__);
  return ret;
}

err_code_t ToolHeadCNC200W::set_output_power(uint8_t new_power, bool is_update_power) {
  if (ctr_mode == CNC_CONSTANT_POWER_MODE) {
    uint8_t run_power = new_power;
    if (is_update_power) {
      if (!set_power(new_power))
        return E_FAILURE;
      run_power = power;
    }
    return sync_cnc_output(run_power);
  }
  else {
    uint16_t run_rpm;
    uint32_t tmp_rpm = new_power * CNC_200W_DEFAULT_MAX_RPM / 100;
    tmp_rpm = tmp_rpm > CNC_200W_DEFAULT_MAX_RPM ? CNC_200W_DEFAULT_MAX_RPM : tmp_rpm;
    run_rpm = (uint16_t)tmp_rpm;
    if (is_update_power) {
      if (!set_target_rpm((uint16_t)tmp_rpm))
        return E_FAILURE;
      run_rpm = target_rpm;
    }
    return sync_cnc_output(run_rpm, CNC_RPM_SET_SPEED);
  }
}

err_code_t ToolHeadCNC200W::set_output_rpm(uint16_t new_rpm, bool is_update_rpm) {
  if (ctr_mode == CNC_CONSTANT_POWER_MODE) {
    uint8_t run_power;
    uint32_t tmp_power = new_rpm * 100 / CNC_200W_DEFAULT_MAX_RPM;
    tmp_power = tmp_power > 100 ? 100 : (uint8_t)tmp_power;
    if (tmp_power == 0 && new_rpm)
      tmp_power = 1;
    run_power = tmp_power;
    if (is_update_rpm) {
      if (!set_power((uint8_t)tmp_power))
        return E_FAILURE;   
      run_power = power; 
    }
    return sync_cnc_output(run_power);
  }
  else {
    uint16_t run_rpm = new_rpm;
    if (is_update_rpm) {
      if (!set_target_rpm(new_rpm))
        return E_FAILURE;
      run_rpm = target_rpm;
    }
    return sync_cnc_output(run_rpm, CNC_RPM_SET_SPEED);
  }
}

err_code_t ToolHeadCNC200W::post_init() {
  LOG_I("HP_CNC post_init in\n");
  uint8_t hw_verion = 0xff;

  if (!get_enclosure_hw_verion(&hw_verion)) {
    LOG_E("HP_CNC GET_HW_VERSION fail\n");
    return E_FAILURE;
  }

  LOG_I("HP_CNC HW_VERSION: 0x%x\n", hw_verion);  

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

  if (module_svc.register_routine((void *)this, hp_cnc_callback_routine)) {
    LOG_E("[%s] HP_CNC register routine func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (register_hmi_command_func(this)) {
    LOG_E("[%s] HP_CNC register hmi command func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (public_mutex_lock()) {
    lost_counter = xTaskGetTickCount();
    online = true;
    set_status(MODULE_STATUS_NORMAL);
    set_hw_version(hw_verion);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] HP_CNC take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  smprinter.register_module(MODULE_DEVICE_ID_CNC_200W_2021, this);
  LOG_I("HP_CNC post_init out\n");
  LOG_I("Hight power CNC ready!\n");
  return E_SUCCESS;
}

err_code_t ToolHeadCNC200W::debug_function(uint8_t cmd, uint32_t param) {
  err_code_t result = E_FAILURE;
  switch(cmd) {
    case CMD_SET_MOTOR_RUN_MODE:
      result = set_run_mode((CNCSpeedControlMode)!!param);
    break;

    case CMD_SET_MOTOR_RUN_DIR:
      result = set_cnc_run_dir(!!param);
    break;

    case CMD_GET_MOTOR_PID_KP:
    case CMD_GET_MOTOR_PID_KI:
    case CMD_GET_MOTOR_PID_KD:
      result = set_cnc_pid(cmd - CMD_GET_MOTOR_PID_KP, 0, param);
    break;

    case CMD_SET_MOTOR_PID_KP:
    case CMD_SET_MOTOR_PID_KI:
    case CMD_SET_MOTOR_PID_KD:
      result = set_cnc_pid(cmd - CMD_SET_MOTOR_PID_KP, 1, param);
    break;

    default:
    break;
  }
  return result;
}
