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
#include "purifier.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_PURIFIER, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_PURIFIER, MODULE_FUNC_PRIORITY_MEDIUM},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

bool Purifier::create_public_mutex_lock() {
  bool ret = true;
  if (public_mutex == NULL) {
    public_mutex = xSemaphoreCreateMutex();
    if (public_mutex == NULL) {
      LOG_E("purifier mutex lock creat fail\n");
      ret = false;
    }
    else
      LOG_I("purifier mutex lock creat success\n");
  }
  else 
    LOG_I("purifier mutex lock already\n");
  return ret;    
}

bool Purifier::public_mutex_lock(uint8_t retry, uint32_t timeout) {
  bool ret = false;
  if (public_mutex) {
    while (retry--) {
      if (xSemaphoreTake(public_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
        ret = true;
        break;
      }
      else 
        LOG_E("take purifier mutex lock fail, retry: %d\n", retry);
    }
  }
  return ret;
}

void Purifier::public_mutex_unlock() {
  if (public_mutex) 
    xSemaphoreGive(public_mutex);
}

err_code_t Purifier::pre_init() {
  LOG_I("Purifier pre_init in\n");
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  // create resource lock
  if (!create_public_mutex_lock())
    return E_FAILURE;

  if (public_mutex_lock()) {
    err = 0;
    fan_cur_out   = 0;
    fan_speed = 0;
    fan_elec = 0;
    fan_gear = PURIFIER_FAN_GEAR_0;
    lifetime  = LIFETIME_INVALID;
    extend_power = 0;
    addon_power = 0;
    tick = 0;
    loop_next_time = 0;
    update_info_flag = 0;
    fan_working_sta = false;
    online = false;
    sysnc_get_info_flag = false;
    sys_sta = 0;
    close_delay_tick = 0;
    set_status(MODULE_STATUS_INIT);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Purifier take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  LOG_I("Purifier pre_init out\n");
  return E_SUCCESS;
}

err_code_t Purifier::deinit() {
  set_fan_control(false);
  if (public_mutex_lock()) {
    fan_cur_out   = 0;
    fan_speed = 0;
    fan_elec = 0;
    extend_power = 0;
    addon_power = 0;
    update_info_flag = 0;
    online = false;
    fan_working_sta = false;
    sys_sta = 0;
    sysnc_get_info_flag = false;
    set_status(MODULE_STATUS_OFFLINE);
    public_mutex_unlock();
  }
  else {
    // prevent state confusion, and set offline if failure
    update_info_flag = 0;
    online = false;
    sysnc_get_info_flag = false;
    set_status(MODULE_STATUS_OFFLINE);
    LOG_E("[%s] Purifier take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  return E_SUCCESS;
}

void Purifier::purifier_offline_check(uint32_t time_out) {
  if (online) {
    if (ELAPSED(xTaskGetTickCount(),tick + time_out)) {
      if (public_mutex_lock()) {
        online = false;
        set_status(MODULE_STATUS_OFFLINE);
        public_mutex_unlock();
      }
      LOG_E("Purifier offline!!!\n");
    }
  }
}

err_code_t Purifier::get_purifier_info(PurifierReportInfoType report_type, bool is_sysnc, uint32_t time_out) {
  smcan_message_t msg;
  err_code_t ret = E_FAILURE;
  uint8_t send_data;
  msg.id = get_message_id(MODULE_FUNC_REPORT_PURIFIER);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get purifier info\n");
    return E_FAILURE;
  }

  if (report_type > PURIFIER_INFO_ALL)
    report_type = PURIFIER_INFO_ALL;
  
  send_data = report_type;
  
  if (is_sysnc) {
    if (public_mutex_lock()) {
      update_info_flag = 0;
      sysnc_get_info_flag = true;
      public_mutex_unlock();
    }
    else {
      LOG_E("[%s] take mutex lock fail\n", __FUNCTION__);
      return E_FAILURE;
    }
  }

  msg.ch     = get_channel();
  msg.data   = &send_data;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (is_sysnc && ret == E_SUCCESS) {
    uint32_t wait_time = time_out;
    uint8_t flag = report_type >= PURIFIER_INFO_ALL ? (~(0xff << PURIFIER_INFO_ALL)) & 0xff : (1 << report_type); 
    while (wait_time > 0) {
      if (flag == update_info_flag)
        break;
      vTaskDelay(pdMS_TO_TICKS(1));
      wait_time--;
    }

    if (flag != update_info_flag) {
      ret = E_EXE_TIMEOUT;
    }
  }

  if (is_sysnc) {
    if (public_mutex_lock()) {
      sysnc_get_info_flag = false;
      public_mutex_unlock();
    }
    else {
      sysnc_get_info_flag = false;
      LOG_E("[%s] set sysnc_get_info_flag fail\n", __FUNCTION__);
    }
    LOG_I("[%s] ret = %d update_info_flag = %d\n", __FUNCTION__, ret, update_info_flag);
  } 
  return ret;  
}

err_code_t purifier_callback_routine(void *obj) {
  Purifier &purifier = *(Purifier *)obj;
  if (obj) {
    if (purifier.online) {
      if (ELAPSED(xTaskGetTickCount(), purifier.loop_next_time + PURIFIER_SAMP_STATUS_INTERVAL)) {
        bool send_close_fan = false;
        purifier.get_purifier_info(PURIFIER_INFO_ALL);
        if (purifier.public_mutex_lock()) {
          purifier.loop_next_time = xTaskGetTickCount();
          if (purifier.close_delay_tick > 0) {
            purifier.close_delay_tick--;
            if (purifier.close_delay_tick == 0)
              send_close_fan = true;
          }
          purifier.public_mutex_unlock();
        }
        else {
          LOG_E("[%s] take mutex lock fail\n", __FUNCTION__);
        }
        if (send_close_fan)
          purifier.set_fan_control(false); 
      }
    }
    purifier.purifier_offline_check();
  }
  return E_SUCCESS;
}

void purifier_callback_update_info(void *obj, uint8_t *data, uint8_t length) {
  Purifier &purifier = *(Purifier *)obj;
  if (!obj || (!purifier.online && !purifier.sysnc_get_info_flag)) 
    return;
  if (purifier.public_mutex_lock()) {
    if (!purifier.online && !purifier.sysnc_get_info_flag) {
      purifier.public_mutex_unlock();
      return;
    }
    bool update = true;
    switch (data[0]) {
      case PURIFIER_INFO_LIFETIME:
        purifier.lifetime = data[1];
      break;

      case PURIFIER_INFO_ERR:
        purifier.err = data[1];
      break;      
      
      case PURIFIER_INFO_FAN_STA:
        purifier.fan_working_sta = !!data[1];
        purifier.fan_speed = (data[2] << 8 | data[3]);
        purifier.fan_cur_out = data[4];
        purifier.fan_gear = data[5];
      break;      
      
      case PURIFIER_INFO_FAN_ELEC:
        purifier.fan_elec = (data[1] << 8 | data[2]);
      break;      
      
      case PURIFIER_REPORT_POWER:
        purifier.addon_power = (data[1] << 8 | data[2]);
        purifier.extend_power = (data[3] << 8 | data[4]);
      break;      
      
      case PURIFIER_REPORT_STATUS:
        purifier.sys_sta = data[1];
      break;

      default:
        update = false;
      break;
    }
    if (update)
      purifier.update_info_flag |= (1 << data[0]);
    purifier.tick = xTaskGetTickCount();
    purifier.public_mutex_unlock();
  }
}

err_code_t Purifier::set_fan_power(uint8_t power) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[2] = {0};
  msg.id = get_message_id(MODULE_FUNC_SET_PURIFIER);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (power > PURIFIER_FAN_MAX_POWER)
    power = PURIFIER_FAN_MAX_POWER;

  buffer[0] = PURIFIER_SET_FAN_POWER;
  buffer[1] = power;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2;
  result = host_can_rou.send(&msg);
  if (result) {
    LOG_E("[%s] send message\n",__FUNCTION__);
  }
  return result;
}

err_code_t Purifier::set_fan_gear(uint8_t gear) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[2] = {0};
  msg.id = get_message_id(MODULE_FUNC_SET_PURIFIER);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (gear > PURIFIER_FAN_GEAR_3)
    gear = PURIFIER_FAN_GEAR_3;

  buffer[0] = PURIFIER_SET_FAN_GEARS;
  buffer[1] = gear;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2;
  result = host_can_rou.send(&msg);
  if (result) {
    LOG_E("[%s] send message\n",__FUNCTION__);
  }
  return result;  
}

err_code_t Purifier::set_light_color(uint8_t red, uint8_t green, uint8_t blue) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint8_t buffer[4] = {0};
  msg.id = get_message_id(MODULE_FUNC_SET_PURIFIER);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  }  
  buffer[0] = PURIFIER_SET_LIGHT;
  buffer[1] = red;
  buffer[2] = green;
  buffer[3] = blue;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 4;
  result = host_can_rou.send(&msg);
  if (result) {
    LOG_E("[%s] send message\n",__FUNCTION__);
  }
  return result;  
}

err_code_t Purifier::set_fan_control(bool is_open, bool is_forced, uint16_t delay_close_s) {
  smcan_message_t msg;
  err_code_t result = E_FAILURE;
  uint32_t close_delay_tick_tmp = 0;
  uint8_t buffer[3] = {0};
  msg.id = get_message_id(MODULE_FUNC_SET_PURIFIER);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("[%s] invalid message\n",__FUNCTION__);
    return E_FAILURE;
  } 
  buffer[0] = PURIFIER_SET_FAN_STA;
  buffer[1] = !!is_open; 
  buffer[2] = !!is_forced; 
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 3;

  if (!is_open) 
    close_delay_tick_tmp = delay_close_s * (1000 / PURIFIER_SAMP_STATUS_INTERVAL);
  else 
    close_delay_tick_tmp = 0;

  if (public_mutex_lock()) {
    close_delay_tick = close_delay_tick_tmp;
    public_mutex_unlock();
    result = E_SUCCESS;
  }
  else {
    LOG_E("[%s] change close_delay_s fail\n", __FUNCTION__);
  }

  if (is_open || (!is_open && close_delay_tick_tmp == 0)) {
    result = host_can_rou.send(&msg);
    if (result) {
      LOG_E("[%s] send message\n",__FUNCTION__);
    }
  }
  return result;  
}

void Purifier::report_purifier_info(void) {
  SnapmakerSettings *sm_settings = NULL;
  LOG_I("purifier %s\n", online ? "online" : "offline");
  LOG_I("purifier error: 0x%x sys_sta: 0x%x\n", err, sys_sta);
  LOG_I("filter lifetime: %s\n", lifetime == LIFETIME_LOW ? "LIFETIME_LOW" : lifetime == LIFETIME_MEDIUM ? \
    "LIFETIME_MEDIUM" : lifetime == LIFETIME_NORMAL ? "LIFETIME_NORMAL" : "LIFETIME_INVALID");
  LOG_I("fan work state: %d\n", fan_working_sta);
  LOG_I("fan speed: %d rpm\n", fan_speed);
  LOG_I("fan out rate: %d%% \n", fan_cur_out);
  LOG_I("fan gears: %d\n", fan_gear);
  LOG_I("fan elec: %d mA\n", fan_elec);
  LOG_I("fan addon power: %d mv\n", addon_power);
  LOG_I("fan extend power: %d mv\n", extend_power);
  sm_settings = smprinter.get_settings();
  if (sm_settings) {
    LOG_I("fdm start work purifier: %s,\tstop work purifier delay %ds close\n", 
      !!(sm_settings->purifier_settings.start_work_purifier_open_mask \
      & (1 << PURIFIER_WORK_TOOL_HEAD_FDM)) ? "auto open" : "no process",\
      sm_settings->purifier_settings.fdm_stop_work_purifier_close_delay);

    LOG_I("laser start work purifier: %s,\tstop work purifier delay %ds close\n", 
      !!(sm_settings->purifier_settings.start_work_purifier_open_mask \
      & (1 << PURIFIER_WORK_TOOL_HEAD_LASER)) ? "auto open" : "no process",\
      sm_settings->purifier_settings.laser_stop_work_purifier_close_delay);

    LOG_I("cnc start work purifier: %s,\tstop work purifier delay %ds close\n", 
      !!(sm_settings->purifier_settings.start_work_purifier_open_mask \
      & (1 << PURIFIER_WORK_TOOL_HEAD_CNC)) ? "auto open" : "no process",\
      sm_settings->purifier_settings.cnc_stop_work_purifier_close_delay);
  }
}

err_code_t send_purifier_info_to_hmi(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  PurifierHeadInfo *tmp_info = NULL;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (purifier.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  msg->data[0] = E_SUCCESS;   // default success
  tmp_info = (PurifierHeadInfo *)(msg->data+1);
  tmp_info->key = purifier.get_key();
  tmp_info->head_status = purifier.get_status();
  tmp_info->info.extend_power = !(purifier.err & PURIFIER_EXTEND_POWER_OFF_MASK);
  tmp_info->info.fan_working_sta = purifier.fan_working_sta;
  tmp_info->info.fan_gear = purifier.fan_gear;
  tmp_info->info.life_time = purifier.lifetime;
  tmp_info->info.filter = !(purifier.err & PURIFIER_NO_FILTER_MASK);
  result = host_hmi.send_ack(msg, msg->data, sizeof(PurifierHeadInfo) + 1);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send msg fail\n",__FUNCTION__);
  }
  return result;  
}

err_code_t hmi_set_purifier_fan_gear(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (purifier.get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("[%s] error state: %d\n",__FUNCTION__, purifier.get_status());
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  LOG_I("[%s]  fan gear %d\n", __FUNCTION__, msg->data[1]);
  result = purifier.set_fan_gear(msg->data[1]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] set fan gear fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   // set fail
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_set_purifier_fan_ctrl(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (purifier.get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("[%s] error state: %d\n",__FUNCTION__, purifier.get_status());
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  LOG_I("[%s]  fan ctrl %d\n", __FUNCTION__, !!msg->data[1]);
  result = purifier.set_fan_control(!!msg->data[1]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] set fan ctrl fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   // set fail
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_set_purifier_start_work_ctrl(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;
  SnapmakerSettings *sm_settings = NULL;
  if (!msg || !obj || msg->length != 3) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("[%s] set work type:%d auto state: %d\n", __FUNCTION__, msg->data[1], msg->data[2]);
  if (msg->data[1] >= PURIFIER_WORK_TOOL_HEAD_INVALID) {
    LOG_E("[%s] error work type: %d\n",__FUNCTION__, msg->data[1]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  sm_settings = smprinter.get_settings();
  if (!sm_settings) {
    LOG_E("[%s] get sm_settings fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  taskENTER_CRITICAL();
  if (msg->data[2]) 
    sm_settings->purifier_settings.start_work_purifier_open_mask |= (1 << msg->data[1]);
  else
    sm_settings->purifier_settings.start_work_purifier_open_mask &= (~(1 << msg->data[1]));
  taskEXIT_CRITICAL();

  motion_platform_svc.save_settings();

  result = host_hmi.send_ack(msg, E_SUCCESS);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_get_purifier_start_work_ctrl(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;
  SnapmakerSettings *sm_settings = NULL;
  bool auto_state = false;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (msg->data[1] >= PURIFIER_WORK_TOOL_HEAD_INVALID) {
    LOG_E("[%s] error work type: %d\n",__FUNCTION__, msg->data[1]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  sm_settings = smprinter.get_settings();
  if (!sm_settings) {
    LOG_E("[%s] get sm_settings fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  auto_state = !!(sm_settings->purifier_settings.start_work_purifier_open_mask & (1 << msg->data[1]));
  msg->data[0] = E_SUCCESS;
  msg->data[1] = auto_state;

  result = host_hmi.send_ack(msg, msg->data, 2);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send msg fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_set_purifier_stop_work_ctrl(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;
  SnapmakerSettings *sm_settings = NULL;
  if (!msg || !obj || msg->length != 4) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("[%s] set work type:%d delay time: %d\n", __FUNCTION__, msg->data[1], *((uint16_t*)(msg->data+2)));
  if (msg->data[1] >= PURIFIER_WORK_TOOL_HEAD_INVALID) {
    LOG_E("[%s] error work type: %d\n",__FUNCTION__, msg->data[1]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  sm_settings = smprinter.get_settings();
  if (!sm_settings) {
    LOG_E("[%s] get sm_settings fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  taskENTER_CRITICAL();    
  if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_FDM)
    sm_settings->purifier_settings.fdm_stop_work_purifier_close_delay = *((uint16_t*)(msg->data+2));
  else if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_LASER)
    sm_settings->purifier_settings.laser_stop_work_purifier_close_delay = *((uint16_t*)(msg->data+2)); 
  else if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_CNC)
    sm_settings->purifier_settings.cnc_stop_work_purifier_close_delay = *((uint16_t*)(msg->data+2));  
  taskEXIT_CRITICAL();

  motion_platform_svc.save_settings();

  result = host_hmi.send_ack(msg, E_SUCCESS);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

err_code_t hmi_get_purifier_stop_work_ctrl(void *obj, sacp_hmi_message_t *msg) {
  Purifier &purifier = *(Purifier *)obj;
  err_code_t result = E_FAILURE;
  SnapmakerSettings *sm_settings = NULL;
  uint16_t delay_times = 0;
  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != purifier.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], purifier.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (msg->data[1] >= PURIFIER_WORK_TOOL_HEAD_INVALID) {
    LOG_E("[%s] error work type: %d\n",__FUNCTION__, msg->data[1]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  sm_settings = smprinter.get_settings();
  if (!sm_settings) {
    LOG_E("[%s] get sm_settings fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_FDM)
    delay_times = sm_settings->purifier_settings.fdm_stop_work_purifier_close_delay;
  else if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_LASER)
    delay_times = sm_settings->purifier_settings.laser_stop_work_purifier_close_delay; 
  else if (msg->data[1] == PURIFIER_WORK_TOOL_HEAD_CNC)
    delay_times = sm_settings->purifier_settings.cnc_stop_work_purifier_close_delay;

  msg->data[0] = E_SUCCESS;
  *((uint16_t *)(msg->data+1)) = delay_times;

  result = host_hmi.send_ack(msg, msg->data, 3);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

uint16_t hmi_subscribe_purifier_func(void *obj, uint8_t *buff) {
  PurifierHeadInfo *tmp_info;
  Purifier &purifier = *(Purifier *)obj;
  if (!obj || !buff) {
    LOG_E("[%s] obj or buffer pointer is null\n",__FUNCTION__);
    return 0;
  }

  buff[0] = E_SUCCESS;
  tmp_info = (PurifierHeadInfo *)(buff + 1);
  tmp_info->key = purifier.get_key();
  tmp_info->head_status = purifier.get_status();
  tmp_info->info.extend_power = !(purifier.err & PURIFIER_EXTEND_POWER_OFF_MASK);
  tmp_info->info.fan_working_sta = purifier.fan_working_sta;
  tmp_info->info.fan_gear = purifier.fan_gear;
  tmp_info->info.life_time = purifier.lifetime;
  tmp_info->info.filter = !(purifier.err & PURIFIER_NO_FILTER_MASK);

  return sizeof(PurifierHeadInfo) + 1;  
}

err_code_t Purifier::register_hmi_command_func(void *obj) {
  err_code_t result = E_FAILURE;
  result = host_hmi.apply_cmd_set_handle(SACP_CMD_SET_AIR_PURIFIER, SACP_CMD_ID_PURIFIER_MAX_NUM);
  if (result != E_SUCCESS && result != E_INVALID_STATE) {
    LOG_E("[%s] apply_cmd_set_handle fail\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_GET_HEAD_INFO, obj, send_purifier_info_to_hmi))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_SET_FAN_GEARS, obj, hmi_set_purifier_fan_gear))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_SET_FAN_ENABLE, obj, hmi_set_purifier_fan_ctrl))
    return E_FAILURE;  
    
  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_SET_HEAD_START_WORK_STA, obj, hmi_set_purifier_start_work_ctrl))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_GET_HEAD_START_WORK_STA, obj, hmi_get_purifier_start_work_ctrl))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_SET_HEAD_STOP_WORK_STA, obj, hmi_set_purifier_stop_work_ctrl))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_AIR_PURIFIER, \
      SACP_CMD_ID_PURIFIER_GET_HEAD_STOP_WORK_STA, obj, hmi_get_purifier_stop_work_ctrl))
    return E_FAILURE;

  if (host_hmi.register_subscription(SACP_CMD_SET_AIR_PURIFIER, SACP_PURIFIER_SUBSCRIBE_COMMANDID, obj, \
      hmi_subscribe_purifier_func))
    return E_FAILURE;

  return E_SUCCESS;
}

void stop_work_notify_purifier_pro(void *obj, uint8_t reason) {
  Purifier &purifier = *(Purifier *)obj;
  SnapmakerSettings *sm_settings = smprinter.get_settings();
  uint16_t delay_s = 0;
  toolHeadType work_type;

  if (!obj) {
    LOG_E("[%s] obj pointer is null\n", __FUNCTION__);
    return;
  }

  if (purifier.check_online()) {
    if (sm_settings) {
      work_type = smprinter.get_toolhead_type();
      if (work_type == TH_TYPE_3DP) {
        delay_s = sm_settings->purifier_settings.fdm_stop_work_purifier_close_delay;
      }
      else if (work_type == TH_TYPE_LASER) {
        delay_s = sm_settings->purifier_settings.laser_stop_work_purifier_close_delay;
      }
      else if (work_type == TH_TYPE_CNC) {
        delay_s = sm_settings->purifier_settings.cnc_stop_work_purifier_close_delay;
      }
    }
    else {
      LOG_E("[%s] get purifier settings fail\n", __FUNCTION__);
    }

    LOG_I("[%s] purifier close delay_s: %d\n", __FUNCTION__, delay_s);
    
    if (delay_s != 0xFFFF)
      purifier.set_fan_control(false, false, delay_s);
    
  }
}

void start_work_notify_purifier_pro(void *obj, uint8_t reason) {
  bool is_open = false;
  Purifier &purifier = *(Purifier *)obj;

  if (!obj) {
    LOG_E("[%s] obj pointer is null\n", __FUNCTION__);
    return;
  }

  if (purifier.check_online()) {
    uint32_t mask = 0;   
    toolHeadType work_type;
    SnapmakerSettings *sm_settings = smprinter.get_settings();
    if (sm_settings) {
      mask = sm_settings->purifier_settings.start_work_purifier_open_mask;
      work_type = smprinter.get_toolhead_type();
      if (work_type == TH_TYPE_3DP) {
        is_open = !!(mask & (1 << PURIFIER_WORK_TOOL_HEAD_FDM));
      }
      else if (work_type == TH_TYPE_LASER) {
        is_open = !!(mask & (1 << PURIFIER_WORK_TOOL_HEAD_LASER));
      }
      else if (work_type == TH_TYPE_CNC) {
        is_open = !!(mask & (1 << PURIFIER_WORK_TOOL_HEAD_CNC));            
      }

      if (is_open)
        purifier.set_fan_control(true);
    }
  }
  LOG_I("[%s] purifier is_open: %d  online: %d\n", __FUNCTION__, is_open, purifier.check_online());
}

err_code_t Purifier::register_notify_handle_func(void *obj) {
  if (job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_STARTED, obj, start_work_notify_purifier_pro))
    return E_FAILURE;

  if (job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_RESUME, obj, start_work_notify_purifier_pro))
    return E_FAILURE;

  if (job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_PAUSED, obj, stop_work_notify_purifier_pro))
    return E_FAILURE;

  if (job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_STOPPED, obj, stop_work_notify_purifier_pro))
    return E_FAILURE;
  return E_SUCCESS;
}

err_code_t Purifier::post_init() {
  LOG_I("Purifier post_init in\n");
  uint16_t msg_id = get_message_id(MODULE_FUNC_REPORT_PURIFIER);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_I("[%s] invalid message MODULE_FUNC_REPORT_PURIFIER\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (host_can_rou.register_callback(msg_id, (void *)this, purifier_callback_update_info) != E_SUCCESS)
    return E_FAILURE;

  if (module_svc.register_routine((void *)this, purifier_callback_routine)) {
    LOG_E("[%s] Purifier register routine func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (get_purifier_info(PURIFIER_INFO_ALL, true)) {
    LOG_E("[%s] sysnc get purifier info fail\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (register_hmi_command_func(this)) {
    LOG_E("[%s] Purifier register hmi command func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (register_notify_handle_func(this)) {
    LOG_E("[%s] Purifier register notify handle func fail\n", __FUNCTION__);
    return E_FAILURE;    
  }

  if (public_mutex_lock()) {
    tick = xTaskGetTickCount();
    loop_next_time = 0;
    online = true;
    set_status(MODULE_STATUS_NORMAL);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Purifier take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  smprinter.register_module(MODULE_DEVICE_ID_PURIFIER_2021, this);
  LOG_I("Purifier post_init out\n");
  LOG_I("Purifier ready\n");
  return E_SUCCESS;
}
