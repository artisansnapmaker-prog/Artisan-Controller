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
#include "../config.h"
#include "../common/debug.h"
#include "../host/sacp_module.h"
#include "../common/utility.h"
#include "../snapmaker.h"
#include "../common/type.h"
#include "system.h"
#include "motion.h"
#include "job_ctrl.h"


JobCtrl job_ctrl_svc;

void JobCtrl::init(void) { 
  _lock = xSemaphoreCreateMutex();
  if (!_lock) {
    LOG_E("_lock create failed\r\n");
    while(1);
  }

  uint8_t *rb_buf = (uint8_t *)pvPortMalloc(GCODE_RB_SIZE);
  if (!rb_buf) {
    LOG_E("can NOT alloc memory for gcode ringbuffer\r\n");
    while(1);
  }
  _gcode_rb.init(rb_buf, (int32_t)GCODE_RB_SIZE);
  _statistics_log_interval_ms = 0;
  _statistics_log_last_tick_ms = 0;
}

void JobCtrl::loop(void) {
  if (!time_after(system_svc.millis(), _tick_ms)) {
    return;
  }

  _tick_ms = system_svc.millis();
  if (JOB_STATUE_PRINTING == _env.status) get_gcodes_from_client();
  if (_statistics_log_interval_ms > 0) {
    if (!time_after(system_svc.millis(), _statistics_log_last_tick_ms + _statistics_log_interval_ms)) {
      statistics_output();
      _statistics_log_last_tick_ms = system_svc.millis();
    } 
  }
  
  LOG_I("Check other event which effect the job\r\n");
}

err_code_t JobCtrl::save_env(void) {
  ModuleBase *cur_toolhead;

  _env.cur_line_num = smprinter.gcode_file_position;
  _env.print_feadrate = smprinter.get_feedrate();
  _env.travel_feadrate = smprinter.get_travl_feedrate();
  _env.g0g1_relative_mode = smprinter.get_relative_mode();
  
  if (TH_TYPE_3DP == _env.type)
    _env.bed_temp = smprinter.get_bet_temp();

  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("can NOT get toolhead\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }
  
  if (!cur_toolhead->save_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("can NOT get toolhead\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::resum_env(void) {
  ModuleBase *cur_toolhead;

  // Check toolhead
  if (smprinter.get_toolhead_type() != _env.type) {

  }

  smprinter.set_feedrate(_env.print_feadrate);
  smprinter.set_travl_feedrate(_env.travel_feadrate);
  smprinter.set_relative_mode(_env.g0g1_relative_mode);

  if (TH_TYPE_3DP == _env.type) {
    if (!smprinter.set_bet_temp(_env.bed_temp)) {
      LOG_E("can NOT resume bed_temp\r\n");
    }
  }

  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("can NOT get toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }
  
  if (!cur_toolhead->save_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("can NOT get toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::machine_standby(void) {
  // TODO:
  return E_SUCCESS;
}

void JobCtrl::clear_gcode_queue(void) {

}

void JobCtrl::notify() {

}

void JobCtrl::quit_stop() {

}

void JobCtrl::normal_stop() {

}

void JobCtrl::get_gcodes_from_client(void) {
  req_batch_gcode_t req_batch_gcode;
  res_batch_gcode_t res_batch_gcode;
  uint8_t batch_gcode_buf[GCODE_RB_SIZE/4];
  
  while(_gcode_rb.available()) {
    req_batch_gcode.line_num = _env.req_line_num;
    req_batch_gcode.buf_len = MIN(_gcode_rb.available(), GCODE_RB_SIZE/4);
    res_batch_gcode.gcode_str = batch_gcode_buf;
    if(ClientNode::get_batch_gcode(_client_id, req_batch_gcode, res_batch_gcode)) {
      if(res_batch_gcode.start_line_num != req_batch_gcode.line_num) {
        LOG_E("start line number not match, drop this batch gcode\r\n");
        _err_get_batch_gcode_cnt++;
        break;
      }
      // shoule we check the line number?
      uint8_t *p = res_batch_gcode.gcode_str;
      uint32_t rx_line_num;
      {
        while('\0' != *p) {
          if ('\n' == *p) {
            rx_line_num++;
          }
          p++;
        }
        if(rx_line_num != (res_batch_gcode.end_line_num - res_batch_gcode.start_line_num)) {
          LOG_E("line number not match, drop this batch gcode\r\n");
          _err_get_batch_gcode_cnt++;
          break;
        }
        _gcode_rb.insert_multi(res_batch_gcode.gcode_str, p - res_batch_gcode.gcode_str);
        _env.req_line_num = res_batch_gcode.end_line_num;
      }
    }
    else {
      _err_get_batch_gcode_cnt++;
    }
  }

  if (_err_get_batch_gcode_cnt > 3) {
    LOG_W("can not get batch gcode from clinet for 3 times, exit working return to idle\r\n");
    stop();
  }
}

void JobCtrl::issue_nodify(void) {
  // report status change reasone
}

void JobCtrl::statistics_output(void) {
  uint32_t rb_size = _gcode_rb.free() + _gcode_rb.available();

  LOG_I("\r\n\r================ job control start ================\r\n");
  LOG_I("gcode ringbuffer: %f free, %f available\r\n",  (float)(100 * _gcode_rb.free()) / rb_size, (float)(100 * _gcode_rb.available()) / rb_size);
  LOG_I("================ job control end ================\r\n");
}

err_code_t JobCtrl::start(uint8_t client_id, struct GcodeFileInfo *gcodeInfo, toolHeadType th_type) {
  // status check
  if (JOB_STATUE_IDLE != _env.status) {
    LOG_E("can not start job as current status is not idle\r\n");
    return E_JOB_NOT_IN_IDLE_STATUS;
  }
  if (motion_svc.sm_homing_needed()) {
    LOG_E("not in home state\r\n");
    return E_JOB_NO_HOME;
  }

  JOB_LOCK();
  // TODO: record this client
  _client_id = client_id;
  _env.type = th_type;
  _env.gcode_file_info = *(gcodeInfo);
  _env.cur_line_num = 0;
  _env.req_line_num = 0;
  _gcode_rb.reset();
  _env.time_elape = 0;
  _env.status = JOB_STATUE_PRINTING;
  _err_get_batch_gcode_cnt = 0;
  JOB_UNLOCK();

  return E_SUCCESS;
}

err_code_t JobCtrl::pause(void) {
  // status check
  if (JOB_STATUE_PRINTING != _env.status || JOB_STATUE_STARTING != _env.status) {
    LOG_E("can NOT pause a job as current status is NOT printing\r\n");
    return E_JOB_NOT_IN_WORKING_STATUS;
  }

  err_code_t ret;
  JOB_LOCK();
  /**
   * We assume all these actions are executable and return true.
   * Should we need to check?
  */
  _env.status = JOB_STATUE_PAUSING;
  if (E_SUCCESS != (ret = save_env())) {
    JOB_UNLOCK();
    return ret;
  }
  clear_gcode_queue();
  normal_stop();
  if (E_SUCCESS != (ret = machine_standby())) {
    JOB_UNLOCK();
    return ret;
  }
  _env.status = JOB_STATUE_PAUSED;
  JOB_UNLOCK();

  return E_SUCCESS;
}


err_code_t JobCtrl::resume(uint8_t client_id) {
  // status check
  if (JOB_STATUE_PAUSED != _env.status) {
    LOG_E("can NOT pause a job as current status is NOT printing\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  JOB_LOCK();
  if (!resum_env()) {
    LOG_E("resume failed\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }
  _client_id = client_id;
  // start gcode request
  JOB_UNLOCK();

  return E_SUCCESS;
}

err_code_t JobCtrl::resume(uint8_t client_id, struct JobEnv &env) {
  // TODO: check env
  _env = env;
  return resume(client_id);
}

err_code_t JobCtrl::stop(void) {
  // Just stop, no matter what the status the machin on
  JOB_LOCK();
  machine_standby();
  _env.status = JOB_STATUE_STOPPED;
  JOB_UNLOCK();
  return E_SUCCESS;
}

err_code_t JobCtrl::set_env(struct JobEnv &env) {
  // TODO: check env
  _env = env;
}

struct JobEnv JobCtrl::get_env(void) {
  return _env;
}

bool JobCtrl::consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  bool ret;
  uint8_t c;
  uint32_t cmd_len;

  ret = false;
  cmd_len = 0;
  JOB_LOCK();
  while(_gcode_rb.remove_one(c)) {    
    if(cmd_len >= max_len) {
      LOG_W("gcode too long for the command buffer");
      ret = false;
      break;
    }

    cmd[cmd_len++] = c;
    if('\n' == c) {
      _env.cur_line_num++;
      *line = _env.cur_line_num;
      cmd[cmd_len] = 0;
      ret = true;
      break;
    }
  }
  JOB_UNLOCK();

  return ret;
}
