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
    LOG_E("job ctrl: _lock create failed\r\n");
    while(1);
  }

  // TODO: should we malloc this buffer use static memory?
  uint8_t *rb_buf = (uint8_t *)pvPortMalloc(GCODE_RB_SIZE);
  if (!rb_buf) {
    LOG_E("can NOT alloc memory for gcode ringbuffer\r\n");
    while(1);
  }
  _gcode_rb.init(rb_buf, (int32_t)GCODE_RB_SIZE);
  _statistics_log_interval_ms = 0;
  _statistics_log_last_tick_ms = 0;
  _env.gfi_valid = false;
  _resume_feedrate = RESUME_FEEDRATE;
}

void JobCtrl::background_thread(void) {
  static uint32_t keep_printing_cnt = 0;
  // if (!time_after(system_svc.millis(), _tick_ms)) {
  //   return;
  // }

  _tick_ms = smprinter.millis();

  if (SYSTEM_STATUS_PRINTING == smprinter.get_sys_status()) {
    keep_printing_cnt++;
    if (keep_printing_cnt >= 3) {
      get_gcodes_from_client();
      keep_printing_cnt = 3;
    }
  }
  else {
    keep_printing_cnt = 0;
  }

  if (_statistics_log_interval_ms > 0) {
    if (!time_after(system_svc.millis(), _statistics_log_last_tick_ms + _statistics_log_interval_ms)) {
      statistics_output();
      _statistics_log_last_tick_ms = smprinter.millis();
    }
  }

  LOG_I("Check other event which effect the job\r\n");
}

err_code_t JobCtrl::save_env(void) {
  ModuleBase *cur_toolhead;

  // if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
  //   LOG_E("Can not get toolhead\r\n");
  //   return E_JOB_SAVE_ENV_FAILURE;
  // }
  LOG_I("TODO: get current_toolhead\r\n");

  /*
  if (!cur_toolhead->save_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("Toolhead save env error\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }
  */
 LOG_I("TODO: current toolhead save env\r\n");

  /*
  if (TH_TYPE_3DP == _env.type)
    _env.bed_temp = motion_svc.get_bet_temp();
  _env.cur_line_num = smprinter.gcode_file_position;
  _env.print_feadrate = motion_svc.get_feedrate();
  _env.travel_feadrate = motion_svc.get_travl_feedrate();
  _env.g0g1_relative_mode = motion_svc.get_relative_mode();
  for(uint32_t i = 0; i < AXIS_NUM; i++)
    _env.current_pos[i] = motion_svc.get_current_position(i);
  */
  LOG_I("TODO: if 3DP, save bed tempretrue\r\n");
  LOG_I("TODO: save current line number\r\n");
  LOG_I("TODO: save print feedrate\r\n");
  LOG_I("TODO: save travle feedrate\r\n");
  LOG_I("TODO: save relative mode\r\n");
  LOG_I("TODO: save current position\r\n");

  return E_SUCCESS;
}

err_code_t JobCtrl::resum_env(void) {
  ModuleBase *cur_toolhead;

  // if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
  //   LOG_E("can not get toolhead\r\n");
  //   return E_JOB_RESUME_ENV_FAILURE;
  // }
  LOG_I("TODO: get current toolhead pointer\r\n");

  // Check toolhead
  // if (smprinter.get_toolhead_type() != _env.type) {
  //   return E_JOB_UNSUPPORT_PARAM;
  // }
  LOG_I("TODO: check current toolhead type\r\n");

  // if (!cur_toolhead->resume_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
  //   LOG_E("can not resume toolhead\r\n");
  //   return E_JOB_RESUME_ENV_FAILURE;
  // }
  LOG_I("TODO: current toolhead resume\r\n");
  

  // if (TH_TYPE_3DP == _env.type) {
  //   thermalManager.setTargetBed(_env.bed_temp);
  //   thermalManager.wait_for_bed();
  // }
  /*
  _env.req_line_num = _env.cur_line_num;
  motion_svc.moveto_xyz(  _env.current_pos[0],
                          _env.current_pos[1],
                          _env.current_pos[2],
                          _resume_feedrate);
  motion_svc.set_feedrate(_env.print_feadrate);
  motion_svc.set_travl_feedrate(_env.travel_feadrate);
  motion_svc.set_relative_mode(_env.g0g1_relative_mode);
  */

  LOG_I("TODO: if 3DP, resume bed tempretrue\r\n");
  LOG_I("TODO: resume current line number\r\n");
  LOG_I("TODO: resume print feedrate\r\n");
  LOG_I("TODO: resume travle feedrate\r\n");
  LOG_I("TODO: resume relative mode\r\n");
  LOG_I("TODO: resume current position\r\n");

  return E_SUCCESS;
}

err_code_t JobCtrl::machine_standby(void) {
  // TODO:
  LOG_I("machine standby begin\r\n");

  // if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
  //   LOG_E("can not get toolhead\r\n");
  //   return E_JOB_RESUME_ENV_FAILURE;
  // }
  LOG_I("TODO: get current toolhead pointer\r\n");

  // Check toolhead
  // if (smprinter.get_toolhead_type() != _env.type) {
  //   return E_JOB_UNSUPPORT_PARAM;
  // }
  LOG_I("TODO: check current toolhead type\r\n");

  switch (_env.type)
  {
  case TH_TYPE_3DP:
    /* code */
    LOG_I("TODO: retrace 10mm in 10mm/s\r\n");
    LOG_I("TODO: hotend set to 0 degree\r\n");
    LOG_I("TODO: fans set to 0 speed\r\n");
    LOG_I("TODO: bed temp set to 0 degree\r\n");

    /*
    motion_svc.set_relative_mode(true);
    motion_svc.moveto_e(-10, 600, true);
    smprinter.fdm->set_fan_speed(0, 0, 0);    // left mode fan
    smprinter.fdm->set_fan_speed(1, 0, 0);    // right mode fan
    smprinter.fdm->set_hotend_temp(0, 0);     // set index 0 hotend
    smprinter.fdm->set_hotend_temp(0, 1);     // set index 1 hotend
    */
    break;

  case TH_TYPE_CNC:
    /* code */
    LOG_I("TODO: retrace 10mm in 10mm/s\r\n");
    LOG_I("TODO: hotend set to 0 degree\r\n");
    LOG_I("TODO: fans set to 0 speed\r\n");
    LOG_I("TODO: bed temp set to 0 degree\r\n");
    break;

  case TH_TYPE_LASER:
    /* code */
    LOG_I("TODO: retrace 10mm in 10mm/s\r\n");
    LOG_I("TODO: hotend set to 0 degree\r\n");
    LOG_I("TODO: fans set to 0 speed\r\n");
    LOG_I("TODO: bed temp set to 0 degree\r\n");
    break;

  default:
    break;
  }

  LOG_I("TODO: Z raise to highest\r\n");
  LOG_I("TODO: x move to left\r\n");
  LOG_I("TODO: y move to head\r\n");

  LOG_I("machine standby end\r\n");
  return E_SUCCESS;
}

void JobCtrl::notify() {

}

void JobCtrl::quit_stop() {
  // motion_svc.quickstop();
}

void JobCtrl::normal_stop() {
  motion_svc.quickstop();
}

void JobCtrl::get_gcodes_from_client(void) {
  req_batch_gcode_t req_batch_gcode;
  res_batch_gcode_t res_batch_gcode;
  uint8_t batch_gcode_buf[GCODE_RB_SIZE/4];
  
  while(_gcode_rb.free()) {
    req_batch_gcode.line_num = _env.req_line_num;
    req_batch_gcode.buf_len = MIN(_gcode_rb.free(), GCODE_RB_SIZE/4);
    res_batch_gcode.gcode_str = batch_gcode_buf;
    if(ClientNode::get_batch_gcode(_client_id, req_batch_gcode, res_batch_gcode)) {
      if(res_batch_gcode.start_line_num != req_batch_gcode.line_num) {
        LOG_E("start line number not match, drop this batch gcode\r\n");
        _err_get_batch_gcode_cnt++;
        break;
      }
      // shoule we check the line number?
      uint8_t *p = res_batch_gcode.gcode_str;
      uint32_t rx_line_num = 0;
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
        // gcode ringbuffer guarantee to hold all the gcode string.
        _gcode_rb.insert_multi(res_batch_gcode.gcode_str, p - res_batch_gcode.gcode_str);
        _env.req_line_num = res_batch_gcode.end_line_num;
        _err_get_batch_gcode_cnt = 0;
      }
    }
    else {
      _err_get_batch_gcode_cnt++;
      break;
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
  if (SYSTEM_STATUS_IDLE != smprinter.get_sys_status() && SYSTEM_STATUS_XY_CALIBRATING != smprinter.get_sys_status()) {
    LOG_E("can not start job as current status is not idle or calibrating\r\n");
    return E_JOB_NOT_IN_IDLE_STATUS;
  }

  if (!gcode_file_info_check(gcodeInfo)) {
    LOG_E("Ivalid gcode file information\r\n");
    return E_JOB_IVALID_GCODE_FILE;
  }

  if (SYSTEM_STATUS_XY_CALIBRATING == smprinter.get_sys_status()) {
    // TODO: calibrating print job
    LOG_I("Start a calibration's printing job\r\n");
    return E_SUCCESS;
  }

  // if (th_type != smprinter.get_toolhead_type()) {
  //   LOG_E("Unmatched toolhead\r\n");
  //   return E_JOB_UNMATCHED_TOOLHEAD;
  // }
  LOG_I("TODO: Check toolhead type\r\n");

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_STARTING, NULL)) {
    LOG_E("Can not enter to SYS_STARTING status\r\n");
    return E_JOB_FAILURE;
  }
  
  LOG_I("TODO: homing\r\n");
  /*
  if (motion_svc.sm_homing_needed()) {
    if(E_SUCCESS != motion_svc.home()) {
      // TODO: do I need to check the result?
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
      return E_JOB_FAILURE;
    }
  }
  */

  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  _client_id = client_id;
  _env.type = th_type;
  _env.gcode_file_info = *(gcodeInfo);
  _env.cur_line_num = 0;
  _env.req_line_num = 0;
  _gcode_rb.reset();
  _env.time_elape = 0;
  _err_get_batch_gcode_cnt = 0;
  _env.gfi_valid = true;
  UNLOCK(_lock);

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PRINTING, NULL)) {
    LOG_E("Can not enter SYS_PRINTING status");
    // TODO: send this exception to system
    return E_JOB_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::pause(void) {
  if (SYSTEM_STATUS_PRINTING != smprinter.get_sys_status()) {
    LOG_E("Can not pause a job as current status is no printing\r\n");
    return E_JOB_NOT_IN_WORKING_STATUS;
  }

  err_code_t ret;
  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSEING, NULL)) {
    LOG_E("Can to enter SYS_PAUSEING status\r\n");
    return E_JOB_FAILURE;
  }

  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  if (E_SUCCESS != (ret = save_env())) {
    UNLOCK(_lock);
    // TODO: do I need to check the result?
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
    return ret;
  }
  normal_stop();
  if (E_SUCCESS != (ret = machine_standby())) {
    UNLOCK(_lock);
    // TODO: do I need to check the result?
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
    return ret;
  }
  _gcode_rb.reset();
  UNLOCK(_lock);

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSED, NULL)) {
    LOG_E("Can not enter SYS_PAUSED status");
    // TODO: send this exception to system
    return E_JOB_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::resume(uint8_t client_id) {
  // status check
  if (SYSTEM_STATUS_PAUSED != smprinter.get_sys_status()) {
    LOG_E("Can resume a job as current status is no pause\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_RESUMING, NULL)) {
    LOG_E("Can to enter SYS_RESUMING status\r\n");
    return E_JOB_FAILURE;
  }

  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  if (E_SUCCESS != resum_env()) {
    UNLOCK(_lock);
    LOG_E("resume failed\r\n");
    // TODO: do I need to check the result?
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
    return E_JOB_RESUME_ENV_FAILURE;
  }

  _client_id = client_id;
  UNLOCK(_lock);

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PRINTING, NULL)) {
    LOG_E("Can not enter SYS_PRINTING status");
    // TODO: send this exception to system
    return E_JOB_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::stop(void) {
  err_code_t ret;
  // status check
  if (SYSTEM_STATUS_PRINTING != smprinter.get_sys_status() && SYSTEM_STATUS_PAUSED != smprinter.get_sys_status()) {
    LOG_E("Can stop a job as current status is no working or paused\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  // Just stop, no matter what the status the machin on
  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  ret = machine_standby();
  UNLOCK(_lock);

  if (E_SUCCESS != ret) {
    LOG_E("machine standby failure\r\n");
    return ret;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL)) {
    LOG_E("Can not enter SYS_IDLE status");
    // TODO: send this exception to system
    return E_JOB_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::set_env(struct JobEnv &env) {
  // TODO: check env
  _env = env;
  return E_SUCCESS;
}

struct JobEnv JobCtrl::get_env(void) {
  return _env;
}

bool JobCtrl::consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  bool ret;
  uint8_t c;
  uint32_t cmd_len;

  if (SYSTEM_STATUS_PRINTING != smprinter.get_sys_status()) {
    return false;
  }

  ret = false;
  cmd_len = 0;
  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  while(_gcode_rb.remove_one(c)) {
    if(cmd_len >= max_len) {
      LOG_W("gcode too long for the command buffer");
      ret = false;
      break;
    }

    cmd[cmd_len++] = c;
    if('\n' == c) {
      *line = _env.cur_line_num;
      _env.cur_line_num++;
      cmd[cmd_len] = 0;
      ret = true;
      break;
    }
  }
  UNLOCK(_lock);

  return ret;
}

bool JobCtrl::gcode_file_info_check(struct GcodeFileInfo *gfi) {
  if(!gfi)
    return false;

  uint8_t *p = gfi->MD5;
  for (uint32_t i = 0; i < GCODE_MD5_LENGTH; i++) {
    if(!( ('a' <= p[i] && p[i] <= 'z') ||
          ('A' <= p[i] && p[i] <= 'Z') ||
          ('0' <= p[i] && p[i] <= '9')
        )
    ) {
      return false;
    }
  }

  p = gfi->name;
  uint32_t nl = 0;
  while(p[nl] != '\0') {
    nl++;
    if(nl > GCODE_FILE_NAME_SIZE){
      return false;
    }
  }

  return true;
}
