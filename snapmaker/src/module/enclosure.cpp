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
#include "enclosure.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "src/core/millis_t.h"
#include "src/HAL/HAL.h"

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
    online = false;
    check_switch = true;
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

// TODO
err_code_t Enclosure::deinit() {
  return E_SUCCESS;
}

void Enclosure::enclosure_offline_check(uint32_t time_out) {
  if (online) {
    if (ELAPSED(xTaskGetTickCount(),tick + time_out)) {
      if (public_mutex_lock()) {
        online = false;
        set_status(MODULE_STATUS_OFFLINE);
        public_mutex_unlock();
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
      if (enclosure.check_switch) {
        // TODO: door open process
        LOG_I("Enclosure door open process\n");
      }
    }
    else {
      LOG_I("Enclosure door close\n");
      if (enclosure.check_switch) {
        // TODO: door close process
        LOG_I("Enclosure door close process\n");
      }
    }
  }
}

bool Enclosure::status_is_change(uint8_t cur_sta, uint8_t old_sta, uint8_t mask) {
  return (cur_sta & mask) ^ (old_sta & mask);
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
  smcan_message_t msg;
  err_code_t ret = E_FAILURE;   
  value = value >= 100 ? 100 : value;
  switch (dev_type) {
    case 0:
      msg.id = get_message_id(MODULE_FUNC_SET_ENCLOSURE_LIGHT);
      if (msg.id ==  MODULE_MESSAGE_ID_INVALID) {
        LOG_E("invalid message to set enclosure light bar\n");
        return ret;
      }
      buffer[i++] = 1;
      buffer[i++] = value;
      buffer[i++] = value;
      buffer[i++] = value;
    break;

    case 1:
      msg.id = get_message_id(MODULE_FUNC_SET_ENCLOSURE_FAN);
      if (msg.id ==  MODULE_MESSAGE_ID_INVALID) {
        LOG_E("invalid message to set enclosure fan\n");
        return ret;
      }
      buffer[i++] = 0;
      buffer[i++] = value;
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

err_code_t Enclosure::enable_enclosure_check() {
  LOG_I("enable enclosure check\n");
  if (public_mutex_lock()) {
     check_switch = true;
     public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Enclosure take public_mutex_lock fail",__FUNCTION__);
    return E_FAILURE;
  }
  // TODO: enable process
  return E_SUCCESS;
}

err_code_t Enclosure::disable_enclosure_check() {
  LOG_I("disable enclosure check\n");
  if (public_mutex_lock()) {
     check_switch = false;
     public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Enclosure take public_mutex_lock fail",__FUNCTION__);
    return E_FAILURE;
  }
  // TODO: disable process
  return E_SUCCESS;
}

void Enclosure::report_enclosure_status() {
  if (online) {
    LOG_I("enclosure check_switch %s\n", check_switch ? "enable" : "disable");
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
  EnclosureInfo *tmp_info = NULL;
  err_code_t result = E_FAILURE;

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

  msg->data[0] = E_SUCCESS;   // default success
  tmp_info = (EnclosureInfo *)(msg->data+1);
  tmp_info->key = enclosure.get_key();
  tmp_info->head_status = enclosure.get_status();;    
  tmp_info->head_active = false;
  tmp_info->light_level = enclosure.light_level;
  tmp_info->check_switch = enclosure.check_switch;
  tmp_info->door_sta = !!(enclosure.enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK);
  tmp_info->fan_speed = enclosure.fan_speed;
  result = host_hmi.send_ack(msg, msg->data, sizeof(EnclosureInfo) + 1);
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

  if (msg->data[1])
    result = enclosure.enable_enclosure_check();
  else 
    result = enclosure.disable_enclosure_check();
  
  if (result != E_SUCCESS) {
    LOG_E("[%s] set enclosure check fail\n",__FUNCTION__);
  }

  msg->data[0] = result == E_SUCCESS ? E_SUCCESS : E_FAILURE;   // set fail
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
  EnclosureInfo *tmp_info;
  Enclosure &enclosure = *(Enclosure *)obj;
  if (!obj || !buffer) {
    LOG_E("[%s] obj or buffer pointer is null\n",__FUNCTION__);
    return 0;
  }

  buffer[0] = E_SUCCESS;
  tmp_info = (EnclosureInfo *)(buffer + 1);
  tmp_info->key = enclosure.get_key();
  tmp_info->head_status = enclosure.get_status();
  tmp_info->head_active = false;
  tmp_info->light_level = enclosure.light_level;
  tmp_info->check_switch = enclosure.check_switch;
  tmp_info->door_sta = !!(enclosure.enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK);
  tmp_info->fan_speed = enclosure.fan_speed;

  return sizeof(EnclosureInfo) + 1;
}

void Enclosure::enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  sacp_hmi_message_t msg;
  EnclosureInfo *tmp_info = NULL;
  uint8_t buff[50];
  switch(test_type) {
    case 0:
      // building messages
      msg.length = 1;
      buff[0] = get_key();
      msg.data = buff;
      send_enclosure_info_to_hmi(this, &msg);
      if (msg.length > 1) {
        tmp_info = (EnclosureInfo *)(msg.data+1);
        LOG_I("send msg len: %d result: %d\n",msg.length, msg.data[0]);
        LOG_I("Enclosure key: %d head_status: %d\n",  tmp_info->key, tmp_info->head_status);
        LOG_I("Enclosure head_active: %d\n",  tmp_info->head_active);
        LOG_I("Enclosure light level: %d  speed: %d\n",  tmp_info->light_level, tmp_info->fan_speed);
        LOG_I("Enclosure check_switch: %s\n",  tmp_info->check_switch ? "enable" : "disable");
        LOG_I("Enclosure door_sta: %s\n",  tmp_info->door_sta ? "open" : "close");
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
        tmp_info = (EnclosureInfo *)(buff + 1);
        LOG_I("send msg len: %d result: %d\n",msg.length, buff[0]);
        LOG_I("Enclosure key: %d head_status: %d\n",  tmp_info->key, tmp_info->head_status);
        LOG_I("Enclosure head_active: %d\n",  tmp_info->head_active);
        LOG_I("Enclosure light level: %d  speed: %d\n",  tmp_info->light_level, tmp_info->fan_speed);
        LOG_I("Enclosure check_switch: %s\n",  tmp_info->check_switch ? "enable" : "disable");
        LOG_I("Enclosure door_sta: %s\n",  tmp_info->door_sta ? "open" : "close");
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
