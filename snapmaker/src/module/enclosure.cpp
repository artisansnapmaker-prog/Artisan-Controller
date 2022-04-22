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
#include "src/HAL/HAL.h"

#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/job_ctrl.h"
#include "enclosure.h"

static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_ENCLOSURE_DOOR_STATE, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_ENCLOSURE_LIGHT, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_ENCLOSURE_FAN, MODULE_FUNC_PRIORITY_LOW},
  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

bool Enclosure::create_public_mutex_lock() {
  bool ret = true;
  if (public_mutex == NULL) {
    public_mutex = xSemaphoreCreateMutex();
    if (public_mutex == NULL) {
      LOG_E("enclosure mutex lock creat fail\n");
      ret = false;
    }
    else
      LOG_I("enclosure mutex lock creat success\n");
  }
  else 
    LOG_I("enclosure mutex lock already\n");
  return ret;    
}

bool Enclosure::public_mutex_lock(uint8_t retry, uint32_t timeout) {
  bool ret = false;
  if (public_mutex) {
    while (retry--) {
      if (xSemaphoreTake(public_mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
        ret = true;
        break;
      }
      else 
        LOG_E("take enclosure mutex lock fail, retry: %d\n", retry);
    }
  }
  return ret;
}

void Enclosure::public_mutex_unlock() {
  if (public_mutex) 
    xSemaphoreGive(public_mutex);
}

err_code_t Enclosure::pre_init() {
  LOG_I("Enclosure pre_init in\n");
  set_func_prio_map(prio_map);

  // create resource lock
  if (!create_public_mutex_lock())
    return E_FAILURE;

  if (public_mutex_lock()) {
    // check_switch = true;
    enclosure_sta = ENCLOSURE_INITIAL_STATE;
    light_level = 0;
    fan_speed = 0;
    loop_next_time = 0;
    tick = xTaskGetTickCount();
    online = false;
    set_status(MODULE_STATUS_INIT);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Enclosure take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  LOG_I("Enclosure pre_init out\n");
  return E_SUCCESS;
}

err_code_t Enclosure::deinit() {
  if (public_mutex_lock()) {
    online = false;
    light_level = 0;
    fan_speed = 0;
    enclosure_sta = ENCLOSURE_INITIAL_STATE;
    set_status(MODULE_STATUS_OFFLINE);
    public_mutex_unlock();
  }
  else {
    // prevent state confusion, and set offline if failure
    online = false;
    set_status(MODULE_STATUS_OFFLINE);
    LOG_E("[%s] Enclosure take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  return E_SUCCESS;
}

void Enclosure::enclosure_offline_check(uint32_t time_out) {
  if (online) {
    if (ELAPSED(xTaskGetTickCount(),tick + time_out)) {
      if (public_mutex_lock()) {
        online = false;
        set_status(MODULE_STATUS_OFFLINE);
        public_mutex_unlock();
        LOG_E("enclosure offline!!!");
      }
    }
  }
}

void enclosure_callback_update_status(void *obj, uint8_t *data, uint8_t length) {
  Enclosure &enclosure = *(Enclosure *)obj;
  if (!obj || !enclosure.online)
    return;
  uint8_t cur_sta = 0;
  bool door_change = false;  
  if (enclosure.public_mutex_lock()) {
    if (!enclosure.online) {
      enclosure.public_mutex_unlock();
      return;
    }

    if (data[0] == ENCLOSURE_DOOR_CLOSE_STATUS)
      cur_sta &= (~ENCLOSURE_DOOR_STATUS_MASK);
    else
      cur_sta |= ENCLOSURE_DOOR_STATUS_MASK;
    
    if (enclosure.enclosure_sta != cur_sta) {
      if (enclosure.status_is_change(cur_sta, enclosure.enclosure_sta, \
        ENCLOSURE_DOOR_STATUS_MASK)) {
        // new door state trigger
        door_change = true;
      }
    }
    enclosure.tick = xTaskGetTickCount();
    // TODO: Modified state better after the event has been successfully 
    // processed, subsequent optimisation.
    enclosure.enclosure_sta = cur_sta;
    enclosure.public_mutex_unlock();
  }

  if (door_change) {
    if (enclosure.enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK) {
      LOG_I("Enclosure door open\n");
      if (enclosure.get_enclosure_check_switch_sta()) {
        // TODO: door open process
        smprinter.pause_trigger(PAUSE_DOOR_OPEN);
      }
    }
    else {
      LOG_I("Enclosure door close\n");
      if (enclosure.get_enclosure_check_switch_sta()) {
        // TODO: door close process
      }
    }
  }
}


uint8_t Enclosure::get_door_check(void) {
  uint8_t ret = 0;
  bool check_switch_ = get_enclosure_check_switch_sta();

  if (check_switch_) {
    if (!online || (enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK))
      ret = 1;
  }
  return ret;
}

bool Enclosure::status_is_change(uint8_t cur_sta, uint8_t old_sta, uint8_t mask) {
  return (cur_sta & mask) ^ (old_sta & mask);
}

uint32_t Enclosure::get_enclosure_check_mask(void) {
  SnapmakerSettings *sm_settings = NULL;
  uint32_t mask = ENCLOSURE_CHECK_ENABLE_DEFAULT_MASK;
  sm_settings = smprinter.get_settings();
  if (sm_settings)
    mask = sm_settings->enclosure_settings.enclosure_check_enable_mask;
  return mask;
}

bool Enclosure::get_enclosure_check_switch_sta(void) {
  bool check_switch_on = false;
  uint32_t mask = get_enclosure_check_mask();
  toolHeadType toolhead = smprinter.get_toolhead_type();

  switch (toolhead) {
    case TH_TYPE_3DP:
      check_switch_on = !!(mask & (1 << ENCLOSURE_WORK_TYPE_FDM));
    break;

    case TH_TYPE_LASER:
      check_switch_on = !!(mask & (1 << ENCLOSURE_WORK_TYPE_LASER));
    break;

    case TH_TYPE_CNC:
      check_switch_on = !!(mask & (1 << ENCLOSURE_WORK_TYPE_CNC));
    break;

    default:
    break;
  }
  return check_switch_on;
}

err_code_t enclosure_callback_routine(void *obj) {
  Enclosure &enclosure = *(Enclosure *)obj;
  if (obj) {
    // query device status regularly
    if (enclosure.online) {
      if (ELAPSED(xTaskGetTickCount(),enclosure.loop_next_time + ENCLOSURE_SAMP_STATUS_INTERVAL)) {
        enclosure.get_enclosure_status();
        if (enclosure.public_mutex_lock()) {
          enclosure.loop_next_time = xTaskGetTickCount();
          enclosure.public_mutex_unlock();
        }
      }
    } 
    // Check if the device is online
    enclosure.enclosure_offline_check();

    // TODO: add loop processing
  }
  return E_SUCCESS;
}

err_code_t Enclosure::set_enclosure_dev_func(uint8_t dev_type, uint8_t value, bool need_ack) {
  uint8_t buffer[8];
  uint8_t out[8];
  uint8_t i = 0;
  uint8_t recv_len = 0;
  uint8_t real_level = 0; 
  smcan_message_t msg;
  err_code_t ret = E_FAILURE;   
  value = value >= 100 ? 100 : value;
  real_level = (uint8_t)(value >= 100 ? 255 : value * 255 / 100);
  switch (dev_type) {
    case 0:
      msg.id = get_message_id(MODULE_FUNC_SET_ENCLOSURE_LIGHT);
      if (msg.id ==  MODULE_MESSAGE_ID_INVALID) {
        LOG_E("invalid message to set enclosure light bar\n");
        return ret;
      }
      buffer[i++] = 1;
      buffer[i++] = real_level;
      buffer[i++] = real_level;
      buffer[i++] = real_level;
    break;

    case 1:
      msg.id = get_message_id(MODULE_FUNC_SET_ENCLOSURE_FAN);
      if (msg.id ==  MODULE_MESSAGE_ID_INVALID) {
        LOG_E("invalid message to set enclosure fan\n");
        return ret;
      }
      buffer[i++] = 0;
      buffer[i++] = real_level;
    break;

    default:
      LOG_E("error enclosure dev type\n");
      return ret;
    break;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = i;

  if (need_ack) {
    ret = host_can_rou.send_sync(&msg, out, &recv_len, 2000);
    if (ret == E_SUCCESS) {
      if (recv_len != 1) {
        LOG_E("set enclosure dev %d\n", dev_type);
        ret = E_FAILURE;
      }
      else {
        ret = out[0] ? E_FAILURE : E_SUCCESS;
        LOG_I("set enclosure dev %d %s\n", dev_type, out[0] ? "fail" : "success");
      }
    }
  }
  else {
    ret = host_can_rou.send(&msg);
  }

  if (ret == E_SUCCESS) {
    if (public_mutex_lock()) {
      if (dev_type == 0) 
        light_level = value;
      else if (dev_type == 1) {
        fan_speed = value;
      }
      public_mutex_unlock();
    }
    else {
      LOG_E("[%s] Enclosure take public_mutex_lock fail",__FUNCTION__);
      return E_FAILURE;
    }
  }
  else {
    LOG_E("dev_type:[%d]  send common fail %s ret=%d\n",dev_type, __FUNCTION__, ret);
  }
  return ret;
}

err_code_t Enclosure::set_light_bar(uint8_t level) {
  return set_enclosure_dev_func(0, level);
}

err_code_t Enclosure::set_fan_speed(uint8_t speed) {
  return set_enclosure_dev_func(1, speed);
}

// err_code_t Enclosure::enable_enclosure_check() {
//   LOG_I("enable enclosure check\n");
//   if (public_mutex_lock()) {
//      check_switch = true;
//      public_mutex_unlock();
//   }
//   else {
//     LOG_E("[%s] Enclosure take public_mutex_lock fail",__FUNCTION__);
//     return E_FAILURE;
//   }
//   // TODO: enable process
//   return E_SUCCESS;
// }

// err_code_t Enclosure::disable_enclosure_check() {
//   LOG_I("disable enclosure check\n");
//   if (public_mutex_lock()) {
//      check_switch = false;
//      public_mutex_unlock();
//   }
//   else {
//     LOG_E("[%s] Enclosure take public_mutex_lock fail",__FUNCTION__);
//     return E_FAILURE;
//   }
//   // TODO: disable process
//   return E_SUCCESS;
// }

void Enclosure::report_enclosure_status() {
  if (online) {
    LOG_I("enclosure check_switch %s\n", get_enclosure_check_switch_sta() ? "enable" : "disable");
    LOG_I("fdm mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_FDM)) ? "enable" : "disable");
    LOG_I("laser mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_LASER)) ? "enable" : "disable");
    LOG_I("cnc mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_CNC)) ? "enable" : "disable");
    LOG_I("enclosure door is %s\n", enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK ? "open" : "close");
    LOG_I("enclosure light bar light_level: %d\n", light_level);
    LOG_I("enclosure light fan speed: %d\n", fan_speed);
  }
  else {
    LOG_I("enclosure offline\n");
  }
}

// Timed queries to get the status of the enclosure module online
err_code_t Enclosure::get_enclosure_status() {
  // uint8_t buffer[8];
  smcan_message_t msg;
  err_code_t ret = E_FAILURE;
  msg.id = get_message_id(MODULE_FUNC_ENCLOSURE_DOOR_STATE);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get enclosure status\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);
  return ret;
}

err_code_t Enclosure::post_init() {
  LOG_I("Enclosure post_init in\n");
  uint16_t msg_id = get_message_id(MODULE_FUNC_ENCLOSURE_DOOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("Enclosure mode invalid message id\n");
    return E_FAILURE;
  }
    
  if (host_can_rou.register_callback(msg_id, (void *)this, enclosure_callback_update_status) != E_SUCCESS) {
    LOG_E("enclosure_callback_update_status func register fail\n");
    return E_FAILURE;
  }

  if (module_svc.register_routine((void *)this, enclosure_callback_routine)) {
    LOG_E("enclosure_callback_routine func register fail\n");
    return E_FAILURE;
  }

  if (register_hmi_command_func(this)) {
    LOG_E("[%s] Enclosure register hmi command func fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  
  get_enclosure_status();

  if (public_mutex_lock()) {
    tick = xTaskGetTickCount();
    loop_next_time = 0;
    online = true;
    set_status(MODULE_STATUS_NORMAL);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Enclosure take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  smprinter.register_module(MODULE_DEVICE_ID_ENCLOSURE_2020, this);
  LOG_I("Enclosure post_init out\n");
  LOG_I("Enclosure ready!!!\n");
  return E_SUCCESS;
}

// functions provided by enclosure for screen use
// 0x15  0x01
err_code_t send_enclosure_info_to_hmi(void *obj, sacp_hmi_message_t *msg) {
  Enclosure &enclosure = *(Enclosure *)obj;
  err_code_t result = E_FAILURE;
  uint16_t i = 0;
  uint32_t mask = 0;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != enclosure.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], enclosure.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (enclosure.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  mask = enclosure.get_enclosure_check_mask(); 

  msg->data[i++] = E_SUCCESS;   // default success
  msg->data[i++] = enclosure.get_key();
  msg->data[i++] = enclosure.get_status();;    
  msg->data[i++] = enclosure.light_level;
  
  msg->data[i++] =  ENCLOSURE_WORK_TYPE_LIMIT;
  for (uint8_t k = 0; k < ENCLOSURE_WORK_TYPE_LIMIT; k++) {
    msg->data[i++] = k;
    msg->data[i++] = !!(mask & (1 << k));
  }
  msg->data[i++] = !!(enclosure.enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK);
  msg->data[i++] = enclosure.fan_speed;

  result = host_hmi.send_ack(msg, msg->data, i); 
  if (result != E_SUCCESS) {
    LOG_E("[%s] send msg fail\n",__FUNCTION__);
  }
  return result;  
}

// 0x15  0x02
err_code_t hmi_set_enclosure_light(void *obj, sacp_hmi_message_t *msg) {
  Enclosure &enclosure = *(Enclosure *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != enclosure.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], enclosure.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  if (enclosure.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }
  LOG_I("[%s]  light level %d\n", __FUNCTION__, msg->data[1]);
  result = enclosure.set_light_bar(msg->data[1]);
  
  if (result != E_SUCCESS) {
    LOG_E("[%s] set enclosure light fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   // set fail
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

// 0x15  0x03
err_code_t hmi_set_enclosure_check(void *obj, sacp_hmi_message_t *msg) {
  Enclosure &enclosure = *(Enclosure *)obj;
  err_code_t result = E_FAILURE;
  SnapmakerSettings *sm_settings = NULL;

  if (!msg || !obj || msg->length != 3) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != enclosure.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], enclosure.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("[%s] work type:%d enable: %d\n", __FUNCTION__, msg->data[1], msg->data[2]);

  if (msg->data[1] >= ENCLOSURE_WORK_TYPE_LIMIT) {
    LOG_E("[%s] error work type: %d\n",__FUNCTION__, msg->data[1]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  sm_settings = smprinter.get_settings();
  if (!sm_settings) {
    LOG_E("[%s] get sm_settings fail\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  taskENTER_CRITICAL();    
  if (msg->data[2])  {
    sm_settings->enclosure_settings.enclosure_check_enable_mask |= (1 << msg->data[1]);
  }
  else 
    sm_settings->enclosure_settings.enclosure_check_enable_mask &= (~(1 << msg->data[1]));
  taskEXIT_CRITICAL();  

  motion_platform_svc.save_settings();
  
  msg->data[0] = E_SUCCESS;   
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

// 0x15  0x04
err_code_t hmi_set_enclosure_fan(void *obj, sacp_hmi_message_t *msg) {
  Enclosure &enclosure = *(Enclosure *)obj;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != enclosure.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], enclosure.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("[%s]  fan level %d\n", __FUNCTION__, msg->data[1]);

  if (enclosure.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  result = enclosure.set_fan_speed(msg->data[1]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] set enclosure set fan speed fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   // set fail
  result = host_hmi.send_ack(msg, msg->data[0]);

  if (result != E_SUCCESS) {
    LOG_E("[%s] send result to hmi fail\n",__FUNCTION__);
  }
  return result;
}

// 0x15  0xa0
uint16_t hmi_subscribe_enclosure_func(void *obj, uint8_t *buffer) {
  Enclosure &enclosure = *(Enclosure *)obj;
  uint16_t i = 0;
  uint32_t mask = 0;

  if (!obj || !buffer) {
    LOG_E("[%s] obj or buffer pointer is null\n",__FUNCTION__);
    return 0;
  }

  mask = enclosure.get_enclosure_check_mask();

  buffer[i++] = E_SUCCESS;
  buffer[i++] = enclosure.get_key();
  buffer[i++] = enclosure.get_status();
  buffer[i++] = enclosure.light_level;
  buffer[i++] =  ENCLOSURE_WORK_TYPE_LIMIT;
  for (uint8_t k = 0; k < ENCLOSURE_WORK_TYPE_LIMIT; k++) {
    buffer[i++] = k;
    buffer[i++] = !!(mask & (1 << k));
  }
  buffer[i++] = !!(enclosure.enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK);
  buffer[i++] = enclosure.fan_speed;

  return i;
}

void Enclosure::enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  sacp_hmi_message_t msg;
  uint8_t buff[50];
  switch(test_type) {
    case 0:
      // building messages
      msg.length = 1;
      buff[0] = get_key();
      msg.data = buff;
      send_enclosure_info_to_hmi(this, &msg);
      if (msg.length > 1) {
        LOG_I("send msg len: %d result: %d\n",msg.length, msg.data[0]);
        LOG_I("Enclosure key: %d head_status: %d\n",  msg.data[1], msg.data[2]);
        for (uint8_t index = 0; index < msg.data[4]; index++) {
          LOG_I("Enclosure work type: %d  enable: %d\n",  msg.data[5+index*2], msg.data[6+index*2]);
        }
        LOG_I("Enclosure light level: %d  speed: %d\n",  msg.data[3], msg.data[msg.data[4] * 2 + 6]);
        LOG_I("Enclosure door_sta: %s\n",  msg.data[msg.data[4] * 2 + 5] ? "open" : "close");
      }
    break;

    case 1:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = (uint8_t)param;
      msg.data = buff;
      hmi_set_enclosure_light(this, &msg);
    break;

    case 2:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = (uint8_t)param;
      msg.data = buff;
      hmi_set_enclosure_check(this, &msg);
    break;

    case 3:
      // building messages
      msg.length = 2;
      buff[0] = get_key();
      buff[1] = (uint8_t)param;
      msg.data = buff;
      hmi_set_enclosure_fan(this, &msg);
    break;

    case 4:
      if (hmi_subscribe_enclosure_func(this, buff)) {
        LOG_I("result: %d\n", buff[0]);
        LOG_I("key: %d head_status: %d\n",  buff[1], buff[2]);
        for (uint8_t index = 0; index < buff[4]; index++) {
          LOG_I("Enclosure work type: %d  enable: %d\n",  buff[5+index*2], buff[6+index*2]);
        }
        LOG_I("Enclosure light level: %d  speed: %d\n",  buff[3], buff[buff[4] * 2 + 6]);
        LOG_I("Enclosure door_sta: %s\n",  buff[buff[4] * 2 + 5] ? "open" : "close");
      }
    break;

    default:
    break;
  }
}

err_code_t Enclosure::register_hmi_command_func(void *obj) {
  err_code_t result = E_FAILURE;
  result = host_hmi.apply_cmd_set_handle(SACP_CMD_SET_ENCLOSURE, SACP_CMD_ID_ENCLOSURE_MAX_NUM);
  if (result != E_SUCCESS && result != E_INVALID_STATE) {
    LOG_E("[%s] apply_cmd_set_handle fail\n",__FUNCTION__);
    return E_FAILURE;
  }
  
  if (host_hmi.register_callback(SACP_CMD_SET_ENCLOSURE, \
      SACP_CMD_ID_ENCLOSURE_GET_HEAD_INFO, obj, send_enclosure_info_to_hmi))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_ENCLOSURE, \
      SACP_CMD_ID_ENCLOSURE_SET_LIGHT_LEVEL, obj, hmi_set_enclosure_light))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_ENCLOSURE, \
      SACP_CMD_ID_ENCLOSURE_ENABLE_CHECK, obj, hmi_set_enclosure_check))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_ENCLOSURE, \
      SACP_CMD_ID_ENCLOSURE_SET_FAN_SPEED, obj, hmi_set_enclosure_fan))
    return E_FAILURE;

  if (host_hmi.register_subscription(SACP_CMD_SET_ENCLOSURE, SACP_ENCLOSURE_SUBSCRIBE_COMMANDID, obj, \
      hmi_subscribe_enclosure_func))
    return E_FAILURE;

  return E_SUCCESS;
}
