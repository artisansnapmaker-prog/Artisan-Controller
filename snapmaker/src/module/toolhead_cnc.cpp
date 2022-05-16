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

#include "src/core/millis_t.h"

#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/job_ctrl.h"
#include "toolhead_cnc.h"

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

bool ToolHeadCNC::create_public_mutex_lock() {
  bool ret = true;
  if (public_mutex == NULL) {
    public_mutex = xSemaphoreCreateMutex();
    if (public_mutex == NULL) {
      LOG_E("cnc mutex lock creat fail\n");
      ret = false;
    }
    else
      LOG_I("cnc mutex lock creat success\n");
  }
  else 
    LOG_I("cnc mutex lock already\n");
  return ret;    
}

bool ToolHeadCNC::public_mutex_lock(uint8_t retry, uint32_t timeout) {
  bool ret = false;
  if (public_mutex) {
    while (retry--) {
      if (xSemaphoreTake(public_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
        ret = true;
        break;
      }
      else 
        LOG_E("take cnc mutex lock fail, retry: %d\n", retry);
    }
  }
  return ret;
}

void ToolHeadCNC::public_mutex_unlock() {
  if (public_mutex) 
    xSemaphoreGive(public_mutex);
}

err_code_t ToolHeadCNC::pre_init() {
  LOG_I("CNC pre_init in\n");
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  // TODO: insertion port of the detection module

  // create resource lock
  if (!create_public_mutex_lock())
    return E_FAILURE;

  if (public_mutex_lock()) {
    power = 0;
    rpm   = 0;
    output_sta = CNC_OUTPUT_OFF;
    ctr_mode  = CNC_CONSTANT_POWER_MODE;
    calibrate_mode = CNC_CALIBRATION_IDLE;
    error_state = 0;
    target_rpm = 0;
    real_power = 0;
    record_error = 0;
    online = false;
    set_status(MODULE_STATUS_INIT);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] CNC take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  LOG_I("CNC pre_init out\n");
  return E_SUCCESS;
}

err_code_t ToolHeadCNC::deinit() {
  if (public_mutex_lock()) {
    real_power = 0;
    rpm   = 0;
    output_sta = CNC_OUTPUT_OFF;
    online = false;
    set_status(MODULE_STATUS_OFFLINE);
    public_mutex_unlock();
  }
  else {
    // prevent state confusion, and set offline if failure
    online = false;
    set_status(MODULE_STATUS_OFFLINE);
    LOG_E("[%s] CNC take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  return E_SUCCESS;
}

err_code_t ToolHeadCNC::prepare_start(void) {
  bool result = false;
  if (get_status() == MODULE_STATUS_NORMAL && !(error_state & CNC_LIMIT_WORK_STATE_MASK) /*&& !smprinter.get_enclosure_door_status()*/)
    result = true;
  else {
    LOG_I("CNC can't start working, status: %d, error_state: 0x%x, door: %d\n", \
      get_status(), error_state, smprinter.get_enclosure_door_status());
  }
  return result;
}

uint8_t ToolHeadCNC::is_can_resume_work(void) {
  uint8_t ret = 0;
  if (get_status() == MODULE_STATUS_NORMAL && !(error_state & CNC_LIMIT_WORK_STATE_MASK))
    ret = 1;
  return ret;
}

// message id callback to handle RPM update from module
void cnc_callback_update_rpm(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  uint8_t stall_trigger = 0;
  if (!obj || !cnc.online)
    return;

  if (cnc.public_mutex_lock()) {
    if (!cnc.online) {
      cnc.public_mutex_unlock();
      return;
    }
    cnc.rpm = (data[0]<<8 | data[1]);
    if (cnc.rpm == 0 && cnc.output_sta == CNC_OUTPUT_OFF_ING) 
      cnc.output_sta = CNC_OUTPUT_OFF;
    
    if (data[2]) {
      // TODO: Modified state better after the event has been successfully 
      // processed, subsequent optimisation.
      if (!(cnc.error_state & CNC_STALL_ERROR_MASK)) {
        // cnc blocking trigger
        stall_trigger = 1;
        cnc.output_sta = CNC_OUTPUT_OFF;
      }
      cnc.error_state |= CNC_STALL_ERROR_MASK;
    }
    else {
      cnc.error_state &= (~CNC_STALL_ERROR_MASK);
    }
    cnc.lost_counter = xTaskGetTickCount();
    cnc.public_mutex_unlock();
  }

  if (stall_trigger) {
    LOG_I("CNC blocking trigger!!!\n");
    cnc.record_error = cnc.error_state;
    smprinter.pause_trigger(PAUSE_EXCEPTION);
  }
}

// cnc module heartbeat detection
void ToolHeadCNC::lost_counter_routine(uint32_t time_out) {
  if (online) {
    if (ELAPSED(xTaskGetTickCount(),lost_counter + time_out)) {
      if (public_mutex_lock()) {
        online = false;
        set_status(MODULE_STATUS_OFFLINE);
        public_mutex_unlock();
        LOG_E("cnc offline!!!");
      }
    }
  }
}

err_code_t cnc_callback_routine(void *obj) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  if (obj)
    cnc.lost_counter_routine();
  return E_SUCCESS;
} 

err_code_t ToolHeadCNC::post_init() {
  LOG_I("CNC post_init in\n");
  uint16_t msg_id = get_message_id(MODULE_FUNC_GET_SPINDLE_SPEED);
  if (msg_id == MODULE_MESSAGE_ID_INVALID)
    return E_FAILURE;

  // register callback to handle RPM from module
  if (host_can_rou.register_callback(msg_id, (void *)this, cnc_callback_update_rpm) != E_SUCCESS)
    return E_FAILURE;
  
  if (module_svc.register_routine((void *)this, cnc_callback_routine)) {
    LOG_E("[%s] CNC register routine func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (register_hmi_command_func(this)) {
    LOG_E("[%s] CNC register hmi command func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (public_mutex_lock()) {
    lost_counter = xTaskGetTickCount();
    online = true;
    set_status(MODULE_STATUS_NORMAL);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] CNC take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  
  motion_platform_svc.set_home_offset(0, 0, 0);
  smprinter.register_module(MODULE_DEVICE_ID_CNC_50W_2019, this);
  LOG_I("CNC post_init out\n");
  LOG_I("CNC ready!\n");
  return E_SUCCESS;
}

bool ToolHeadCNC::set_power(uint8_t new_power) {
  bool ret = false;
  if (public_mutex_lock()) {
    if (new_power > CNC_POWER_MAX)
      new_power = CNC_POWER_MAX;
    power = real_power = new_power;   // The default and target power are the same 
    public_mutex_unlock();
    ret = true;
  }
  else 
    LOG_E("[%s] CNC take public_mutex_lock fail\n", __FUNCTION__);
  return ret;
}

err_code_t ToolHeadCNC::set_output_power(uint8_t new_power, bool is_update_power) {
  uint8_t run_power = new_power;
  if (is_update_power) {
    if (!set_power(new_power)) {
      return E_FAILURE;
    }
    run_power = power;
  }
  return sync_cnc_output(run_power);
}

bool ToolHeadCNC::set_calibrate_mode(CNCCalibrationMode mode) {
  bool ret = false;
  if (mode > CNC_CALIBRATION_IDLE) {
    LOG_E("[%s] invalid parameter \n", __FUNCTION__);
    return false;
  }
  
  if (public_mutex_lock()) {
    calibrate_mode = mode;
    public_mutex_unlock();
    ret = true;
  }
  else {
    LOG_E("[%s] CNC take public_mutex_lock fail \n", __FUNCTION__);
  }
  return ret;
}

void ToolHeadCNC::report_cnc_status_info() {
  LOG_I("CNC rpm: %d, error: 0x%x\n",rpm, error_state);
  LOG_I("CNC key: %d, CNC ctr mode: %s\n", get_key(), ctr_mode ? "CONSTANT_RPM_MODE" : "CONSTANT_POWER_MODE");
  LOG_I("CNC run status: %s\n",output_sta == 0 ? "STOP" :  output_sta == 1 ? "RUN" : "STOPING");
  LOG_I("CNC cur_power: %d target_power: %d\n",  real_power, power);
  LOG_I("CNC cur_rpm: %d target_rpm: %d\n",  rpm, target_rpm);
  LOG_I("CNC calibration mode: %d\n",calibrate_mode);
  LOG_I("CNC mode status: %d\n",get_status());
  LOG_I("CNC last error: 0x%x\n", record_error);
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

  if (public_mutex_lock()) {
    if (value > 0)
      output_sta = CNC_OUTPUT_ON;
    else if (output_sta == CNC_OUTPUT_ON)
      output_sta = CNC_OUTPUT_OFF_ING;
    else 
      output_sta = CNC_OUTPUT_OFF;
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] cnc take public_mutex_lock fail\n", __FUNCTION__);
  }
  return E_SUCCESS;
}


err_code_t ToolHeadCNC::save_env(uint8_t *env_buf, uint32_t &len) {
  err_code_t result = E_FAILURE;
  uint32_t need_len = sizeof(CNCToolHeadInfo) + 4;
  uint32_t check_sum = 0;
  CNCToolHeadInfo *tmp_info = NULL;
  tmp_info = (CNCToolHeadInfo *)env_buf;
  if (len >= need_len) {
    tmp_info->key = get_key();
    tmp_info->head_status = get_status();    
    tmp_info->head_active = false;
    tmp_info->control_mode = ctr_mode;
    tmp_info->run_state = output_sta;
    if (output_sta == CNC_OUTPUT_ON)
      tmp_info->cur_power = real_power;
    else 
      tmp_info->cur_power = 0;
    tmp_info->target_power = power;
    tmp_info->cur_rpm = rpm;
    tmp_info->target_rpm = target_rpm;

    //add simple calibration, 
    for (uint32_t i = 0; i < sizeof(CNCToolHeadInfo); i++) {
      check_sum += env_buf[i];
    }
    check_sum ^= 0x20;
    memcpy(env_buf+sizeof(CNCToolHeadInfo), (uint8_t*)(&check_sum), 4);
    result = E_SUCCESS;
    len = sizeof(CNCToolHeadInfo) + 4;
  }
  else {
    len = 0;
  } 
  return result;
}

err_code_t ToolHeadCNC::resume_env(uint8_t *env_buf, uint32_t &len) {
  err_code_t result = E_FAILURE;
  uint32_t check_sum = 0;
  uint32_t tmp_sum = 0;
  CNCToolHeadInfo *tmp_info = NULL;
  if (len == sizeof(CNCToolHeadInfo) + 4) {
    for (uint32_t i = 0; i < sizeof(CNCToolHeadInfo); i++) {
      tmp_sum += env_buf[i];
    }
    tmp_sum ^= 0x20;
    check_sum = *(uint32_t*)(env_buf + sizeof(CNCToolHeadInfo));
    if (tmp_sum != check_sum) {
      LOG_E("[%s] cnc info check sum error, read check_sum:0x%x cal check_sum: 0x%x\n",__FUNCTION__,check_sum, tmp_sum);
      goto resume_out;
    }
    tmp_info = (CNCToolHeadInfo *)env_buf;
    LOG_I("CNC key: %d head_status: %d\n",  tmp_info->key, tmp_info->head_status);
    LOG_I("CNC head_active: %d control_mode: %s\n",  tmp_info->head_active, \
          tmp_info->control_mode ? "CONSTANT_RPM_MODE" : "CONSTANT_POWER_MODE");
    LOG_I("CNC run_state: %s\n",  tmp_info->run_state == 0 ? "STOP" : \
          tmp_info->run_state == 1 ? "RUN" : "STOPING");
    LOG_I("CNC cur_power: %d target_power: %d\n",  tmp_info->cur_power, tmp_info->target_power);
    LOG_I("CNC cur_rpm: %d target_rpm: %d\n",  tmp_info->cur_rpm, tmp_info->target_rpm);

    if (tmp_info->control_mode != ctr_mode && is_support_change_ctr_mode()) {
      if (set_run_mode((CNCSpeedControlMode)tmp_info->control_mode)) {
        LOG_E("[%s] resume ctr_mode fail.\n",__FUNCTION__);
        goto resume_out;
      }
    }

    if (tmp_info->target_power != power) {
      if (!set_power(tmp_info->target_power)) {
        LOG_E("[%s] resume run power fail.\n",__FUNCTION__);
        goto resume_out;
      }
    }

    if (tmp_info->target_rpm != target_rpm && is_support_rpm_mode()) {
      if (!set_target_rpm(tmp_info->target_rpm)) {
        LOG_E("[%s] resume run rpm fail.\n",__FUNCTION__);
        goto resume_out;
      }
    }

    if (tmp_info->run_state == CNC_OUTPUT_ON) {
      if (ctr_mode == CNC_CONSTANT_POWER_MODE) 
        result = sync_cnc_output(power, CNC_PWM_SET_SPEED);
      else 
        result = sync_cnc_output(target_rpm, CNC_RPM_SET_SPEED);
    }
    else {
      result = sync_cnc_output(0, CNC_PWM_SET_SPEED);
    }

    if (result) {
      LOG_E("[%s] sync_cnc_output fail\n",__FUNCTION__);
    }
  }
  else {
    LOG_E("[%s] error cnc info len\n",__FUNCTION__);
  }

resume_out:
  return result;
}

err_code_t ToolHeadCNC::standby(void) {
  return sync_cnc_output(0, CNC_PWM_SET_SPEED);
}

err_code_t ToolHeadCNC::start_spindle_self_test(void) {
  LOG_I("[%s] the current module does not support self-testing\n");
  return E_FAILURE;
}

// functions provided by CNC for screen use
// commandset 0x11  commandId 0x01 
err_code_t send_cnc_head_info_to_hmi(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  CNCToolHeadInfo *tmp_info = NULL;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != cnc.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], cnc.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  msg->data[0] = E_SUCCESS;   // default success
  tmp_info = (CNCToolHeadInfo *)(msg->data+1);
  tmp_info->key = cnc.get_key();
  tmp_info->head_status = cnc.get_status();    
  tmp_info->head_active = false;
  tmp_info->control_mode = cnc.ctr_mode;
  tmp_info->run_state = cnc.output_sta;
  if (cnc.output_sta == CNC_OUTPUT_ON)
    tmp_info->cur_power =  cnc.real_power;
  else 
    tmp_info->cur_power = 0;
  tmp_info->target_power =  cnc.power;
  tmp_info->cur_rpm = cnc.rpm;
  tmp_info->target_rpm = cnc.target_rpm;
  result = host_hmi.send_ack(msg, msg->data, sizeof(CNCToolHeadInfo) + 1);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send msg fail\n",__FUNCTION__);
  }

  return result;
}

// commandset 0x11  commandId 0x02 
err_code_t hmi_set_cnc_power(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != cnc.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], cnc.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (cnc.set_power(msg->data[1]))
    result = E_SUCCESS;


  if (result == E_SUCCESS &&  cnc.output_sta == CNC_OUTPUT_ON \
    && cnc.ctr_mode == CNC_CONSTANT_POWER_MODE) {
    result = cnc.sync_cnc_output(cnc.power);
  }

  if (result != E_SUCCESS) {
    LOG_E("[%s] set spindle power fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE; 
  result = host_hmi.send_ack(msg, msg->data[0]);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

// commandset 0x11  commandId 0x03 
err_code_t hmi_set_cnc_rpm(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  uint32_t tmp_rpm = 0;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 5) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != cnc.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], cnc.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (!cnc.is_support_rpm_mode()) {
    LOG_E("[%s] No support for setting speed\n",__FUNCTION__);
    msg->data[0] = E_CNC_NO_SUPPORT;      // no support
  }
  else {
    tmp_rpm = *((uint32_t*)(msg->data + 1));
    LOG_I("hmi set rpm %d\n", tmp_rpm);
    tmp_rpm = tmp_rpm > 0xffff ? 0xffff : tmp_rpm;
    if (cnc.set_target_rpm((uint16_t)tmp_rpm))
      result = E_SUCCESS;

    if (result == E_SUCCESS &&  cnc.output_sta == CNC_OUTPUT_ON \
        && cnc.ctr_mode == CNC_CONSTANT_RPM_MODE) {
      result = cnc.sync_cnc_output(cnc.target_rpm, CNC_RPM_SET_SPEED);
    }

    if (result != E_SUCCESS) {
      LOG_E("[%s] set spindle rpm fail\n",__FUNCTION__);
    }

    msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   
  }
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

// commandset 0x11  commandId 0x04 
err_code_t hmi_set_cnc_ctr_mode(void *obj, sacp_hmi_message_t *msg) { 
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != cnc.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], cnc.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (!cnc.is_support_change_ctr_mode()) {
    LOG_E("[%s] No support for setting ctr_mode\n",__FUNCTION__);
    msg->data[0] = E_CNC_NO_SUPPORT;      // no support
  }
  else {
    if (cnc.online && cnc.output_sta != CNC_OUTPUT_ON) {
      result = cnc.set_run_mode((CNCSpeedControlMode)(!!(msg->data[1])));
      if (result != E_SUCCESS) {
        LOG_E("[%s] set_run_mode fail\n", __FUNCTION__);
      }
      msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;
    }
    else {
      msg->data[0] = E_INVALID_STATE;
    }
  }

  result = host_hmi.send_ack(msg, msg->data[0]);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_set_cnc_enable(void *obj, sacp_hmi_message_t *msg) { 
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != cnc.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, inconsistency between\n",\
      __FUNCTION__, msg->data[0], cnc.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (cnc.online) {
    if (msg->data[1]) {
      if (cnc.ctr_mode == CNC_CONSTANT_POWER_MODE) {
        result = cnc.sync_cnc_output(cnc.power, CNC_PWM_SET_SPEED);
      }
      else {
        result = cnc.sync_cnc_output(cnc.target_rpm, CNC_RPM_SET_SPEED);
      }
    }
    else 
      result = cnc.sync_cnc_output(0);
  }
  else {
    result = E_INVALID_STATE;
  }

  if (result != E_SUCCESS) {
    LOG_E("[%s] sync_cnc_output fail\n", __FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_set_cnc_enter_calibrate(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (msg->data[0] >= CNC_CALIBRATION_IDLE) {
    LOG_E("[%s] error cnc calibrate param [%d]\n",__FUNCTION__,msg->data[0]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  LOG_I("cnc enter calibrate mode. parm: %d\n", msg->data[0]);

  if (smprinter.set_sys_status(SYSTEM_STATUS_CNC_CALIBRATING, NULL)) {
    LOG_E("[%s] cnc enter calibrate mode fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_FAILURE);
  }

  if (!cnc.set_calibrate_mode((CNCCalibrationMode)msg->data[0])) {
    LOG_E("[%s] cnc set_calibrate_mode calibrate mode fail\n", __FUNCTION__);
  }

  return host_hmi.send_ack(msg, E_SUCCESS);
}

err_code_t hmi_set_cnc_exit_calibrate(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (cnc.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  LOG_I("cnc exit calibrate mode. parm: %d\n", msg->data[0]);

  if (smprinter.get_sys_status() != SYSTEM_STATUS_CNC_CALIBRATING) {
    LOG_E("[%s] not currently in CNC calibration state, exit failed \n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_FAILURE);
  }

  if (smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL)) {
    LOG_E("[%s] cnc enter calibrate mode fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_FAILURE);
  }

  if (!cnc.set_calibrate_mode(CNC_CALIBRATION_IDLE)) {
    LOG_E("[%s] cnc set_calibrate_mode calibrate mode fail\n", __FUNCTION__);
  }

  return host_hmi.send_ack(msg, E_SUCCESS);
}


uint16_t hmi_subscribe_cnc_func(void *obj, uint8_t *buffer) {
  CNCSpeedState *speed_sta = NULL;
  ToolHeadCNC &cnc = *(ToolHeadCNC *)obj;
  if (!obj || !buffer) {
    LOG_E("[%s] obj or buffer pointer is null\n",__FUNCTION__);
    return 0;
  }
  buffer[0] = E_SUCCESS;    // default Success
  speed_sta = (CNCSpeedState *)(buffer + 1);
  speed_sta->key = cnc.get_key();
  speed_sta->run_state = cnc.output_sta;
  if (cnc.output_sta == CNC_OUTPUT_ON) 
    speed_sta->cur_power = cnc.real_power;
  else 
    speed_sta->cur_power = 0;
  speed_sta->target_power = cnc.power;
  speed_sta->cur_rpm = cnc.rpm;
  speed_sta->target_rpm = cnc.target_rpm;
  speed_sta->control_mode = cnc.ctr_mode;
  return sizeof(CNCSpeedState) + 1;
}

void ToolHeadCNC::cnc_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  sacp_hmi_message_t msg;
  CNCSpeedState *tmp_state = NULL;
  CNCToolHeadInfo *tmp_info = NULL;
  uint8_t buff[50];
  switch(test_type) {
    case 0:
      // building messages
      msg.length = 1;
      buff[0] = get_key();
      msg.data = buff;
      send_cnc_head_info_to_hmi(this, &msg);
      if (msg.length > 1) {
        tmp_info = (CNCToolHeadInfo *)(msg.data + 1);
        LOG_I("send msg len: %d result: %d\n",msg.length, msg.data[0]);
        LOG_I("CNC key: %d head_status: %d\n",  tmp_info->key, tmp_info->head_status);
        LOG_I("CNC head_active: %d control_mode: %s\n",  tmp_info->head_active, \
              tmp_info->control_mode ? "CONSTANT_RPM_MODE" : "CONSTANT_POWER_MODE");
        LOG_I("CNC run_state: %s\n",  tmp_info->run_state == 0 ? "STOP" : \
              tmp_info->run_state == 1 ? "RUN" : "STOPING");
        LOG_I("CNC cur_power: %d target_power: %d\n",  tmp_info->cur_power, tmp_info->target_power);
        LOG_I("CNC cur_rpm: %d target_rpm: %d\n",  tmp_info->cur_rpm, tmp_info->target_rpm);        
      }
    break;

    case 1:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = (uint8_t)param;
      msg.data = buff;
      hmi_set_cnc_power(this, &msg);
    break;

    case 2:
      // building messages
      msg.length = 5;
      buff[0] = get_key();
      buff[1] = (param >> 0) & 0xff;
      buff[2] = (param >> 8) & 0xff;
      buff[3] = (param >> 16) & 0xff;
      buff[4] = (param >> 24) & 0xff;
      msg.data = buff;
      hmi_set_cnc_rpm(this, &msg);
    break;

    case 3:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = !!param;
      msg.data = buff;
      hmi_set_cnc_ctr_mode(this, &msg);
    break;

    case 4:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = !!param;
      msg.data = buff;
      hmi_set_cnc_enable(this, &msg);
    break;

    case 5:
      if (hmi_subscribe_cnc_func(this, buff)) {
        tmp_state = (CNCSpeedState *)(buff + 1);
        LOG_I("CNC result: %d\n",  buff[0]);
        LOG_I("CNC key: %d\n",  tmp_state->key);
        LOG_I("CNC run_state: %s\n",  tmp_state->run_state == 0 ? "STOP" : \
            tmp_state->run_state == 1 ? "RUN" : "STOPING");
        LOG_I("CNC cur_power: %d target_power: %d\n",  tmp_state->cur_power, tmp_state->target_power);
        LOG_I("CNC cur_rpm: %d target_rpm: %d\n",  tmp_state->cur_rpm, tmp_state->target_rpm);
        LOG_I("CNC control_mode: %s\n",  tmp_state->control_mode ? "CONSTANT_RPM_MODE" : "CONSTANT_POWER_MODE");
      }
    break;

    case 6: 
      msg.length = 1;
      buff[0] = (uint8_t)param;
      msg.data = buff;
      hmi_set_cnc_enter_calibrate(this, &msg);
    break;

    case 7: 
      msg.length = 1;
      buff[0] = (uint8_t)param;
      msg.data = buff;
      hmi_set_cnc_exit_calibrate(this, &msg);
    break;

    // case 8:
    //   len_ = 50;
    //   save_env(test_buf, len_);
    //   LOG_I("test_len = %d\n",len_);
    // break;

    // case 9:
    //   resume_env(test_buf, len_);
    //   LOG_I("test_len = %d\n",len_);
    // break;

    default:
    break;
  }
}

err_code_t ToolHeadCNC::register_hmi_command_func(void *obj) {
  err_code_t result = E_FAILURE;
  result = host_hmi.apply_cmd_set_handle(SACP_CMD_SET_CNC, SACP_CMD_ID_CNC_MAX_NUM);
  if (result != E_SUCCESS && result != E_INVALID_STATE) {
    LOG_E("[%s] apply_cmd_set_handle fail\n",__FUNCTION__);
    return E_FAILURE;
  }

  result = host_hmi.apply_cmd_set_handle(SACP_CMD_SET_CALIBRATE_CNC, SACP_CMD_ID_CNC_CALIBRATE_NUM_MAX);
  if (result != E_SUCCESS && result != E_INVALID_STATE) {
    LOG_E("[%s] apply_cmd_set_handle fail\n",__FUNCTION__);
    return E_FAILURE;
  }
  
  if (host_hmi.register_callback(SACP_CMD_SET_CNC, \
      SACP_CMD_ID_CNC_GET_HEAD_INFO, obj, send_cnc_head_info_to_hmi))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CNC, \
      SACP_CMD_ID_CNC_SET_POWER, obj, hmi_set_cnc_power))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CNC, \
      SACP_CMD_ID_CNC_SET_RPM, obj, hmi_set_cnc_rpm))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CNC, \
      SACP_CMD_ID_CNC_SET_CTR_MODE, obj, hmi_set_cnc_ctr_mode))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CNC, \
      SACP_CMD_ID_CNC_SET_ENABLE, obj, hmi_set_cnc_enable))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_CNC, \
      SACP_CMD_ID_CNC_ENTER_CALIBRATE, obj, hmi_set_cnc_enter_calibrate))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_CNC, \
      SACP_CMD_ID_CNC_EXIT_CALIBRATE, obj, hmi_set_cnc_exit_calibrate))
    return E_FAILURE;

  if (host_hmi.register_subscription(SACP_CMD_SET_CNC, SACP_CNC_SUBSCRIBE_COMMANDID, obj, \
      hmi_subscribe_cnc_func))
    return E_FAILURE;

  return E_SUCCESS;
}
