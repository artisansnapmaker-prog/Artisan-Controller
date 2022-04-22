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
#include "enclosure_a400.h"

static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_ENCLOSURE_DOOR_STATE, MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_ENCLOSURE_LIGHT, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_ENCLOSURE_FAN, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_GET_HW_VERSION, MODULE_FUNC_PRIORITY_LOW},
  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t EnclosureA400::pre_init() {
  LOG_I("Enclosure A400 pre_init in\n");
  set_func_prio_map(prio_map);

  // create resource lock
  if (!create_public_mutex_lock())
    return E_FAILURE;
  
  if (public_mutex_lock()) {
    // check_switch = true;
    enclosure_sta = ENCLOSURE_INITIAL_STATE;
    light_level = 0;
    fan_speed = 0;
    light_adc = 0;
    tick = xTaskGetTickCount(); 
    online = false;
    set_status(MODULE_STATUS_INIT);
    public_mutex_unlock();  
  } 
  else {
    LOG_E("[%s] Enclosure A400 take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }
  LOG_I("Enclosure A400 pre_init out\n");
  return E_SUCCESS;
}

err_code_t EnclosureA400::set_light_bar(uint8_t level) {
  // return set_enclosure_dev_func(0, level, true);
  return set_enclosure_dev_func(0, level);
}

err_code_t EnclosureA400::set_fan_speed(uint8_t speed) {
  // return set_enclosure_dev_func(1, speed, true);
  return set_enclosure_dev_func(1, speed);
}

void EnclosureA400::report_enclosure_status() {
  if (online) {
    LOG_I("enclosure check %s\n", get_enclosure_check_switch_sta() ? "enable" : "disable");
    LOG_I("fdm mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_FDM)) ? "enable" : "disable");
    LOG_I("laser mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_LASER)) ? "enable" : "disable");
    LOG_I("cnc mode check_switch %s\n",(get_enclosure_check_mask() & \
                        (1 << ENCLOSURE_WORK_TYPE_CNC)) ? "enable" : "disable");
    LOG_I("enclosure sta: 0x%x\n", enclosure_sta);
    LOG_I("enclosure door is %s\n", enclosure_sta & ENCLOSURE_DOOR_STATUS_MASK ? "open" : "close");
    LOG_I("enclosure light bar light_level: %d\n", light_level);
    LOG_I("enclosure light fan speed: %d\n", fan_speed);

    LOG_I("enclosure light check limit %s\n", enclosure_sta & \
      ENCLOSURE_LIGHT_LIMIT_STATUS_MASK ? "enable" : "disable");
    LOG_I("enclosure hall 1 status %s\n", enclosure_sta & ENCLOSURE_HALL_1_STATUS_MASK ? "hight" : "low");
    LOG_I("enclosure hall 2 status %s\n", enclosure_sta & ENCLOSURE_HALL_2_STATUS_MASK ? "hight" : "low");
    LOG_I("enclosure light adc:  %d\n", light_adc);
  }
  else {
    LOG_I("enclosure offline\n");
  }
}

void enclosure_a400_callback_update_status(void *obj, uint8_t *data, uint8_t length) {
  EnclosureA400 &enclosure = *(EnclosureA400 *)obj;
  uint8_t cur_sta = 0;
  bool door_change = false;
  bool light_limit = false;
  // Reserved, enable the following code if more outer cover state handling is required
  // bool door1_change = false;
  // bool door2_change = false;
  // bool door_close = false;
  if (!obj || !enclosure.online)
    return;
  // door staus
  if (data[0] == ENCLOSURE_DOOR_CLOSE_STATUS)
    cur_sta &= (~ENCLOSURE_DOOR_STATUS_MASK);
  else
    cur_sta |= ENCLOSURE_DOOR_STATUS_MASK;
  // light limit
  if (data[1]) 
    cur_sta |= ENCLOSURE_LIGHT_LIMIT_STATUS_MASK; 
  else
    cur_sta &= (~ENCLOSURE_LIGHT_LIMIT_STATUS_MASK);
  // hall 1
  if (data[2]) 
    cur_sta |= ENCLOSURE_HALL_1_STATUS_MASK; 
  else
    cur_sta &= (~ENCLOSURE_HALL_1_STATUS_MASK);
  // hall 2
  if (data[3]) 
    cur_sta |= ENCLOSURE_HALL_2_STATUS_MASK; 
  else
    cur_sta &= (~ENCLOSURE_HALL_2_STATUS_MASK);

  if (enclosure.public_mutex_lock()) {
    if (!enclosure.online) {
      enclosure.public_mutex_unlock();
      return;
    }
    uint8_t old_sta = enclosure.enclosure_sta;
    if (cur_sta != old_sta) {
      if (enclosure.status_is_change(cur_sta, old_sta, ENCLOSURE_DOOR_STATUS_MASK)) {
        door_change = true;
      }

      // Reserved, enable the following code if more outer cover state handling is required
      // if (enclosure.status_is_change(cur_sta, old_sta, ENCLOSURE_HALL_1_STATUS_MASK | ENCLOSURE_HALL_2_STATUS_MASK)) {
      //   if (!!(cur_sta & ENCLOSURE_DOOR_STATUS_MASK) == ENCLOSURE_DOOR_CLOSE_STATUS) {
      //     // TODO: door close process
      //     door_close = true;
      //   }
      //   else {
      //     // check door 1 is change
      //     if (enclosure.status_is_change(cur_sta, old_sta, ENCLOSURE_HALL_1_STATUS_MASK)) {
      //       door1_change = true;
      //     }

      //     // check door 2 is change
      //     if (enclosure.status_is_change(cur_sta, old_sta, ENCLOSURE_HALL_2_STATUS_MASK)) {
      //       door2_change = true;
      //     }
      //   }
      // }

      if (enclosure.status_is_change(cur_sta, old_sta, ENCLOSURE_LIGHT_LIMIT_STATUS_MASK)) {
        light_limit = true;
      }
    }

    enclosure.light_adc = (data[4] << 8 | data[5]);
    enclosure.tick = xTaskGetTickCount();
    // TODO: Modified state better after the event has been successfully 
    // processed, subsequent optimisation.
    enclosure.enclosure_sta = cur_sta;
    enclosure.public_mutex_unlock();
  }
  
  if (door_change) {
    if (cur_sta & ENCLOSURE_DOOR_STATUS_MASK) {
      LOG_I("Enclosure door open\n");
      if (enclosure.get_enclosure_check_switch_sta()) {
        // TODO: door open process
        LOG_I("Enclosure door open process\n");
        smprinter.pause_trigger(PAUSE_DOOR_OPEN);
      }
    }
    else {
      LOG_I("Enclosure door close\n");
      if (enclosure.get_enclosure_check_switch_sta()) {
        // TODO: door open process
      }
    }
  }
  
  // Reserved, enable the following code if more outer cover state handling is required
  // if (door_close) {
  //   LOG_I("all enclosure doors closed\n");
  // }
  // else {
  //   if (door1_change) {
  //     if (!!(cur_sta & ENCLOSURE_HALL_1_STATUS_MASK) == ENCLOSURE_DOOR_CLOSE_STATUS) {
  //       LOG_I("Enclosure door 1 close\n");
  //       // TODO: door 1 close process
  //       if (enclosure.check_switch) {
  //         LOG_I("Enclosure door 1 close process\n");
  //       }
  //     }
  //     else {
  //       LOG_I("Enclosure door 1 open\n");
  //       // TODO: door 1 open process
  //       if (enclosure.check_switch) {
  //         LOG_I("Enclosure door 1 open process\n");
  //       }
  //     }
  //   }

  //   if (door2_change) {
  //     if (!!(cur_sta & ENCLOSURE_HALL_2_STATUS_MASK) == ENCLOSURE_DOOR_CLOSE_STATUS) {
  //       LOG_I("Enclosure door 2 close\n");
  //       // TODO: door 2 close process
  //       if (enclosure.check_switch) {
  //         LOG_I("Enclosure door 2 close process\n");
  //       }
  //     }
  //     else {
  //       LOG_I("Enclosure door 2 open\n");
  //       // TODO: door 2 open process
  //       if (enclosure.check_switch) {
  //         LOG_I("Enclosure door 2 open process\n");
  //       }
  //     }
  //   }
  // }

  if (light_limit) {
    if (cur_sta & ENCLOSURE_LIGHT_LIMIT_STATUS_MASK) {
      LOG_I("Enclosure light bar limit enable\n");
      // TODO: light bar limit enable process
      if (enclosure.get_enclosure_check_switch_sta()) {
        LOG_I("Enclosure light bar limit enable process\n");
      }
    }
    else {
      LOG_I("Enclosure light bar limit disable\n");
      //  TODO: light bar limit disable process
      if (enclosure.get_enclosure_check_switch_sta()) {
        LOG_I("Enclosure light bar limit disable process\n");
      }
    }
  }
}

err_code_t enclosure_a400_callback_routine(void *obj) {
  EnclosureA400 &enclosure = *(EnclosureA400 *)obj;
  if (obj) {
    enclosure.enclosure_offline_check();
  }
  return E_SUCCESS;
}

bool EnclosureA400::get_enclosure_hw_verion(uint8_t *version) {
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

err_code_t EnclosureA400::post_init() {
  LOG_I("Enclosure A400 post_init in\n");
  uint8_t hw_verion = 0xff;

  if (!get_enclosure_hw_verion(&hw_verion)) {
    LOG_E("Enclosure A400 GET_HW_VERSION fail\n");
    return E_FAILURE;
  }

  LOG_I("Enclosure A400 HW_VERSION: 0x%x\n", hw_verion);  

  uint16_t msg_id = get_message_id(MODULE_FUNC_ENCLOSURE_DOOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("Enclosure a400 mode invalid message id\n");
    return E_FAILURE;
  }

  if (host_can_rou.register_callback(msg_id, (void *)this, enclosure_a400_callback_update_status) != E_SUCCESS) {
    LOG_E("enclosure_a400_callback_update_status func register fail\n");
    return E_FAILURE;
  }

  if (module_svc.register_routine((void *)this, enclosure_a400_callback_routine)){
    LOG_E("[%s] Enclosure A400 register routine func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (register_hmi_command_func(this)) {
    LOG_E("[%s] Enclosure A400 register hmi command func fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  if (public_mutex_lock()) {
    tick = xTaskGetTickCount();
    online = true;
    set_status(MODULE_STATUS_NORMAL);
    set_hw_version(hw_verion);
    public_mutex_unlock();
  }
  else {
    LOG_E("[%s] Enclosure A400 take public_mutex_lock fail\n", __FUNCTION__);
    return E_FAILURE;
  }

  smprinter.register_module(MODULE_DEVICE_ID_ENCLOSURE_A400_2022, this);
  LOG_I("Enclosure A400 post_init out\n");
  LOG_I("Enclosure a400 ready!!!\n");
  return E_SUCCESS;
}
