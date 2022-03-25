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

void job_ctrl_thread_entry(void *p) {
  for(;;) {
    job_ctrl_svc.background_thread(p);
  }
}

void JobCtrl::init(void) { 
  BaseType_t ret;
  uint8_t *rb_buf;

  _lock = xSemaphoreCreateMutex();
  configASSERT(_lock);

  _req_queue = xMessageBufferCreate(JOB_CTRL_REQ_INFO_BUF);
  configASSERT(_req_queue);

  rb_buf = (uint8_t *)pvPortMalloc(GCODE_RB_SIZE);
  configASSERT(rb_buf);
  _gcode_rb.init(rb_buf, (int32_t)GCODE_RB_SIZE);

  rb_buf = (uint8_t *)pvPortMalloc(4);
  configASSERT(rb_buf);
  _issue_ret_rb.init(rb_buf, 4);

  _statistics_log_interval_ms = 0;
  _statistics_log_last_tick_ms = 0;
  _env.gfi_valid = false;

  ret = xTaskCreate((TaskFunction_t)(job_ctrl_thread_entry), "jobctrl", SYSTEM_TASK_STACK_SIZE,
        (void *)(this), HIGHEST_TASK_PRIORITY,  NULL);
  if (ret != pdPASS) {
    LOG_E("job_ctrl: cant not create thread\r\n");
    while(1);
  }
}

void JobCtrl::background_thread(void *p) {
  static uint32_t keep_printing_cnt = 0;
  JobCtrlReqInfo jri;
  size_t len;

  len = xMessageBufferReceive(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(JOB_CTRL_LOOP_TIME_MS));
  if (len == sizeof(jri)) {
    switch (jri.req_action)
    {
    case REQ_START:
      do_start(jri);
      break;

    case REQ_PAUSE:
      do_pause(jri);
      break;

    case REQ_RESUME:
      do_resume(jri);
      break;

    case REQ_STOP:
      do_stop(jri);
      break;
    
    default:
      break;
    }
  }
  
  // make sure send starting ACK before get gcodes
  // TODO: uncomment when release
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
      _statistics_log_last_tick_ms = system_svc.millis();
    }
  }
  
  // TODO: check other event such as temperature protetion, stall protection
  
  if (SYSTEM_STATUS_FINISHING == smprinter.get_sys_status() && 
      _gcode_rb.is_empty() ){
    LOG_I("push all gcodes to marlin or other 3D printer\r\n");
    issue_nodify(E_JOB_ISSUE_RET_FINISH);
    req_stop();
  }

  while (_issue_ret_rb.available()) {
    uint8_t issue_ret;
    _issue_ret_rb.remove_one(issue_ret);
    issue_nodify(issue_ret);
  }
}

err_code_t JobCtrl::req_start(  uint8_t client_id, 
                                struct GcodeFileInfo *gcodeInfo, 
                                toolHeadType th_type, 
                                job_req_notify_cb_t cb/* = NULL*/, 
                                void *p/* = NULL*/) {
  SystemStatus s = smprinter.get_sys_status();
  if (SYSTEM_STATUS_IDLE != s && 
      SYSTEM_STATUS_XY_CALIBRATING != s) {
    LOG_E("can not start job as current status is not idle or calibrating\r\n");
    return E_JOB_NOT_IN_IDLE_STATUS;
  }

  if (SYSTEM_STATUS_XY_CALIBRATING == s) {
    LOG_I("Start a calibration's printing job\r\n");
  }

  if (th_type != smprinter.get_toolhead_type()) {
    LOG_E("job_ctrl: Unmatched toolhead\r\n");
    return E_JOB_UNMATCHED_TOOLHEAD;
  }

  if (!gcode_file_info_check(gcodeInfo)) {
    LOG_E("Ivalid gcode file information\r\n");
    return E_JOB_IVALID_GCODE_FILE;
  }

  JobCtrlReqInfo jri;
  jri.req_action = REQ_START;
  jri.req_data.req_start_data.client_id = client_id;
  jri.req_data.req_start_data.gcodeInfo = *gcodeInfo;
  jri.req_data.req_start_data.th_type = th_type;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::req_pause( enum JobPauseType pt, 
                                      job_req_notify_cb_t cb/* = NULL*/, void *p/* = NULL*/) {
  if (SYSTEM_STATUS_PRINTING != smprinter.get_sys_status()) {
    LOG_E("job client: can not pause a job as current status is no printing\r\n");
    return E_JOB_NOT_IN_WORKING_STATUS;
  }

  JobCtrlReqInfo jri;
  jri.req_action = REQ_PAUSE;
  jri.req_data.req_pause_data.type = pt;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::req_resume( uint8_t client_id, 
                                job_req_notify_cb_t cb/* = NULL*/, 
                                void *p/* = NULL*/) {
  // status check
  if (SYSTEM_STATUS_PAUSED != smprinter.get_sys_status()) {
    LOG_E("job_ctrl: Can not resume a job as current status is no pause\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  JobCtrlReqInfo jri;
  jri.req_action = REQ_RESUME;
  jri.req_data.req_resume_data.client_id = client_id;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::req_stop(job_req_notify_cb_t cb/* = NULL*/, void *p/* = NULL*/) {
  SystemStatus s = smprinter.get_sys_status();
  if (SYSTEM_STATUS_PRINTING != s && 
      SYSTEM_STATUS_PAUSED != s &&
      SYSTEM_STATUS_FINISHING != s &&
      SYSTEM_STATUS_XY_CALIBRATING_PRINTING != s) {
    LOG_E("job_ctrl: Can not stop a job as current status is no working or paused\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  JobCtrlReqInfo jri;
  jri.req_action = REQ_STOP;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

void JobCtrl::print_job_env(struct JobEnv *env) {
  LOG_I("TYPE: %d\r\n", env->type);
  LOG_I("gcode name: %s\r\n", env->gcode_file_info.name);
  LOG_I("req_line_num: %d\r\n", env->req_line_num);
  LOG_I("cur_line_num: %d\r\n", env->cur_line_num);
  LOG_I("cur_pos:\r\n");
  for(uint32_t i = 0; i < AXIS_NUM; i++) LOG_I("cur_pos[%d]: %f\r\n", i, _env.current_pos[i]);
  LOG_I("print_feadrate: %f\r\n", env->print_feadrate);
  LOG_I("travel_feadrate: %f\r\n", env->travel_feadrate);
  LOG_I("g0g1_relative_mode: %d\r\n", env->g0g1_relative_mode);
  LOG_I("bed_temp: %d\r\n", env->bed_temp);
  LOG_I("toolhead_env_buf_size: %d\r\n", env->toolhead_env_buf_size);
  for (uint32_t i = 0; i < env->toolhead_env_buf_size; i++) LOG_I("%02X ", env->toolhead_env_buf[i]);
}

err_code_t JobCtrl::save_env(void) {
  ModuleBase *cur_toolhead;

  LOG_I("job_ctrl: get current_toolhead\r\n");
  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: Can not get toolhead\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }

  LOG_I("TODO: job_ctrl: current toolhead save env\r\n");
  if (TH_TYPE_3DP != smprinter.get_toolhead_type()) {
    // TODO: no do save_env for 3dp toolhead as 3dp toolhead save_env has bugs.
    _env.toolhead_env_buf_size = MODULE_ENV_MAX_SIZE;
    if (E_SUCCESS != cur_toolhead->save_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
      LOG_E("Toolhead save env error\r\n");
      return E_JOB_SAVE_ENV_FAILURE;
    }
  }

  LOG_I("job_ctrl: if 3DP, save bed tempretrue, call the bed module's save_env\r\n");
  LOG_I("job_ctrl: save current line number\r\n");
  LOG_I("job_ctrl: save print feedrate\r\n");
  LOG_I("job_ctrl: save travle feedrate\r\n");
  LOG_I("job_ctrl: save relative mode\r\n");
  LOG_I("job_ctrl: save current position\r\n");
  
  if (TH_TYPE_3DP == _env.type){
    ModuleBase *bed;
    bed = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (bed) {
      _env.bed_env_buf_size = MODULE_ENV_MAX_SIZE;
      if (E_SUCCESS != bed->save_env(_env.bed_env_buf, _env.bed_env_buf_size)) {
        LOG_I("job_ctrl: bed save env failure\r\n");
      }
      else {
        LOG_I("job_ctrl: bed_temp save\r\n");
      }
    }
    else {
      LOG_I("job_ctrl: can not get bed\r\n");
    }
  }    
  _env.active_coordinate = motion_svc.get_active_coordinate_system();
  _env.cur_line_num = smprinter.gcode_file_position;
  LOG_I("job_ctrl: cur_line_num %d\r\n", _env.cur_line_num);
  _env.print_feadrate = motion_svc.get_feedrate();
  _env.travel_feadrate = motion_svc.get_travl_feedrate();
  _env.g0g1_relative_mode = motion_svc.get_relative_mode();
  for(uint32_t i = 0; i < AXIS_NUM; i++)
    _env.current_pos[i] = motion_svc.get_current_position(i);
  
  print_job_env(&_env);
  return E_SUCCESS;
}

err_code_t JobCtrl::resum_env(void) {
  ModuleBase *cur_toolhead;

  LOG_I("job_ctrl: get current toolhead pointer\r\n");
  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: can not get toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  // Check toolhead
  LOG_I("job_ctrl: check current toolhead type\r\n");
  if (smprinter.get_toolhead_type() != _env.type) {
    return E_JOB_UNSUPPORT_PARAM;
  }

  LOG_I("TODO: current toolhead resume\r\n");
  if (TH_TYPE_3DP != smprinter.get_toolhead_type()) {
    // TODO: no do resume for 3dp toolhead as 3dp toolhead resume_env has bugs.
    if (E_SUCCESS != cur_toolhead->resume_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
      LOG_E("job_ctrl: can not resume toolhead\r\n");
      return E_JOB_RESUME_ENV_FAILURE;
    } 
  }
  
  if (TH_TYPE_3DP == _env.type) {
    ModuleBase *bed;
    bed = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (bed) {
      if (E_SUCCESS != bed->resume_env(_env.bed_env_buf, _env.bed_env_buf_size)) {
        LOG_I("job_ctrl: bed resume env failure\r\n");
      }
      else {
        LOG_I("job_ctrl: bed resume\r\n");
      }
    }
    else {
      LOG_I("job_ctrl: can not get bed\r\n");
    }
  }
  
  LOG_I("job_ctrl: if 3DP, resume bed tempretrue\r\n");
  LOG_I("job_ctrl: resume current line number\r\n");
  LOG_I("job_ctrl: resume print feedrate\r\n");
  LOG_I("job_ctrl: resume travle feedrate\r\n");
  LOG_I("job_ctrl: resume relative mode\r\n");
  LOG_I("job_ctrl: resume current position\r\n");
  
  _env.req_line_num = _env.cur_line_num;
  LOG_I("job_ctrl: req_line_num %d\r\n", _env.req_line_num);
  
  /*
  LOG_I("job_ctrl: resume coordinate\r\n");
  if (_env.active_coordinate == -1) {
    motion_svc.run_gcode("G53", true);
    LOG_I("job_ctrl: resume to G53\r\n");
  }
  else if (_env.active_coordinate == 0) {
    motion_svc.run_gcode("G54", true);
    LOG_I("job_ctrl: resume to G54\r\n");
  }
  else {
    LOG_E("job_ctrl: Unknow coordinate\r\n");
  }
  */

  LOG_I("job_ctrl: resume relative mode %d\r\n", _env.g0g1_relative_mode);
  motion_svc.set_relative_mode(_env.g0g1_relative_mode);
  
  LOG_I("job_ctrl: resume xy position %f %f\r\n", _env.current_pos[0], _env.current_pos[1]);
  motion_svc.moveto_xy(_env.current_pos[0], _env.current_pos[1], RESUME_XY_FEEDRATE);

  LOG_I("job_ctrl: resume z position %f\r\n", _env.current_pos[2]);
  motion_svc.moveto_z(_env.current_pos[2], RESUME_Z_FEEDRATE);

  LOG_I("job_ctrl: resume print_feadrate %f\r\n", _env.print_feadrate);
  motion_svc.set_feedrate(_env.print_feadrate);

  LOG_I("job_ctrl: resume travel_feadrate %f\r\n", _env.travel_feadrate);
  motion_svc.set_travl_feedrate(_env.travel_feadrate);

  LOG_I("job_ctrl: resume relative mode %d\r\n", _env.g0g1_relative_mode);
  motion_svc.set_relative_mode(_env.g0g1_relative_mode);

  return E_SUCCESS;
}

err_code_t JobCtrl::machine_standby(void) {
  ModuleBase *cur_toolhead;
  LOG_I("%d job_ctrl: machine standby begin\r\n", millis());

  LOG_I("job_ctrl: get current toolhead pointer\r\n");
  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: can not get toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  LOG_I("job_ctrl: check current toolhead type\r\n");
  if (smprinter.get_toolhead_type() != _env.type) {
    LOG_E("job_ctrl: toolhead not macth\r\n");
    return E_JOB_UNSUPPORT_PARAM;
  }

  if (E_SUCCESS != cur_toolhead->standby()) {
    LOG_E("job_ctrl: toolhead standby\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  switch (_env.type)
  {
  case TH_TYPE_3DP:
    /* code */
    // motion_svc.set_relative_mode(true);
    // motion_svc.moveto_e(-10, 600, true);
    // smprinter.set_fdm_fan_speed(0, 0); // left mode fan
    // smprinter.set_fdm_fan_speed(1, 0); // right mode fan
    // smprinter.set_hotend_temp(0, 0); // set index 0 hotend
    // smprinter.set_hotend_temp(0, 1); // set index 1 hotend
    break;

  case TH_TYPE_CNC:
    /* code */
    break;

  case TH_TYPE_LASER:
    /* code */
    break;

  default:
    break;
  }

  LOG_I("TODO: retrace 10mm in 10mm/s\r\n");
  LOG_I("TODO: hotend set to 0 degree\r\n");
  LOG_I("TODO: fans set to 0 speed\r\n");
  LOG_I("TODO: bed temp set to 0 degree\r\n");

  // motion_svc.run_gcode("G55");
  LOG_I("job_ctrl: Z raise to highest\r\n");
  // motion_svc.run_gcode("G28 Z", true);
  // motion_svc.run_gcode("G0 Z400 F3000", true);
  motion_svc.update_position_from_platform();
  xyze_pos_t t_pos =  motion_svc.sm_current_position;
  t_pos.z = motion_svc.get_max_position(Z_AXIS);
  motion_svc.moveto(t_pos, 30, true);

  // X do not need to standby
  // LOG_I("job_ctrl: x move to left\r\n");
  // motion_svc.run_gcode("G28 X", true);
  // motion_svc.run_gcode("G0 X5", true);

  LOG_I("job_ctrl: y move to head\r\n");
  // motion_svc.run_gcode("G28 Y", true);
  // motion_svc.run_gcode("G0 Y400 F3000", true);
  motion_svc.update_position_from_platform();
  t_pos = motion_svc.sm_current_position;
  t_pos.y = motion_svc.get_max_position(Y_AXIS);
  motion_svc.moveto(t_pos, 30, true);

  LOG_I("job_ctrl: machine standby end\r\n");
  return E_SUCCESS;
}

void JobCtrl::notify() {

}

void JobCtrl::get_gcodes_from_client(void) {
  req_batch_gcode_t req_batch_gcode;
  res_batch_gcode_t res_batch_gcode;
  uint8_t batch_gcode_buf[GCODE_RB_SIZE/4];
  
  while(_gcode_rb.free() >= _get_gcode_buffer_req_min) {
    req_batch_gcode.line_num = _env.req_line_num;
    req_batch_gcode.buf_len = (uint16_t) (MIN((uint32_t)_gcode_rb.free(), GCODE_RB_SIZE/4));
    if (req_batch_gcode.buf_len < _get_gcode_buffer_req_min){
      LOG_I("job_ctrl: no large enough buffer for get gcode, minimum request %d, but we juest get %d\r\n", _get_gcode_buffer_req_min, req_batch_gcode.buf_len);
      break;
    }
    res_batch_gcode.gcode_str = batch_gcode_buf;
    LOG_I("job_ctrl: get gcode from client %d, startline %d, buffer %d\r\n", _client_id, req_batch_gcode.line_num, req_batch_gcode.buf_len);
    if(ClientNode::get_batch_gcode(_client_id, req_batch_gcode, res_batch_gcode)) {
      if (E_SUCCESS != res_batch_gcode.result &&
          E_JOB_LAST_GCODE_PACK != res_batch_gcode.result) {
        LOG_I("job_ctrl: get gcode's result error\r\n");
        _err_get_batch_gcode_cnt++;
        continue;
      }

      if(res_batch_gcode.start_line_num != req_batch_gcode.line_num) {
        LOG_E("start line number not match, drop this batch gcode\r\n");
        _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER);
        req_stop();
        break;
      }
      // shoule we check the line number?
      uint8_t *p, *ls;
      p = ls = res_batch_gcode.gcode_str;
      uint8_t str_temp[MAX_CMD_SIZE];
      uint32_t rx_line_num = 0;
      {
        while('\0' != *p) {
          if ('\n' == *p) {
            rx_line_num++;
            // 747 debug
            if (p - ls < MAX_CMD_SIZE) {
              memcpy(str_temp, ls, (p - ls));
              str_temp[p-ls] = 0;
              ls = p + 1;
              // LOG_I("job_ctrl: get gocde: %s\r\n", (char *)str_temp);
            }
          }
          p++;
        }

        if (0 == rx_line_num) {
          LOG_I("job_ctrl: get 0 line gcode, perhaps no large buffer for the next gcode, break and wait for the next large gcode buffer\r\n");
          // update 
          _get_gcode_buffer_req_min = req_batch_gcode.buf_len + 1;
        }

        if (rx_line_num != ((res_batch_gcode.end_line_num - res_batch_gcode.start_line_num) + 1)) {
          LOG_E("line number not match, drop this batch gcode, expect %d, but get %d\r\n", rx_line_num, ((res_batch_gcode.end_line_num - res_batch_gcode.start_line_num) + 1));
          _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER);
          req_stop();
          break;
        }
        // gcode ringbuffer guarantee to hold all the gcode string.
        _gcode_rb.insert_multi(res_batch_gcode.gcode_str, p - res_batch_gcode.gcode_str);
        _env.req_line_num = res_batch_gcode.end_line_num + 1;
        _err_get_batch_gcode_cnt = 0;

        if (E_JOB_LAST_GCODE_PACK == res_batch_gcode.result) {
          LOG_I("job_ctrl: Job control get last gcode packe\r\n");
          if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_FINISHING, NULL)) {
            // we will continue to request the client to send gcodes
            LOG_E("job_ctrl: can to enter SYS_FINISHING status\r\n");
          }
          break;
        }
      }
    }
    else {
      _err_get_batch_gcode_cnt++;
      break;
    }
  }

  if (_err_get_batch_gcode_cnt > 3) {
    LOG_W("can not get batch gcode from clinet for 3 times, exit working return to idle\r\n");
    _issue_ret_rb.insert_one(E_JOB_ISSUE_RET_GET_GCODE_FAILURE);
    req_stop();
  }
}

void JobCtrl::issue_nodify(uint8_t issue_ret) {
  // report status change reasone
  LOG_I("job_ctrl: issue %d\r\n", issue_ret);
  ClientNode::issue_client(_client_id, issue_ret);
}

void JobCtrl::statistics_output(void) {
  uint32_t rb_size = _gcode_rb.free() + _gcode_rb.available();

  LOG_I("\r\n\r================ job control start ================\r\n");
  LOG_I("gcode ringbuffer: %f free, %f available\r\n",  (float)(100 * _gcode_rb.free()) / rb_size, (float)(100 * _gcode_rb.available()) / rb_size);
  LOG_I("================ job control end ================\r\n");
}

void JobCtrl::do_start(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (SYSTEM_STATUS_XY_CALIBRATING == smprinter.get_sys_status()) {
    // TODO: ???
  }
  else {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_STARTING, &ret_sys_status)) {
      LOG_E("job_ctrl: Can not enter to SYS_STARTING status\r\n");
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      return;
    }
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_STARTING);
  }

  _client_id = jri.req_data.req_start_data.client_id;
  _env.type = jri.req_data.req_start_data.th_type;
  _env.gcode_file_info = jri.req_data.req_start_data.gcodeInfo;
  _env.cur_line_num = 0;
  _env.req_line_num = 0;
  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  _gcode_rb.reset();
  UNLOCK(_lock);
  _env.time_elape = 0;
  _err_get_batch_gcode_cnt = 0;
  _env.gfi_valid = true;
  _get_gcode_buffer_req_min = 0;

  if (SYSTEM_STATUS_XY_CALIBRATING == smprinter.get_sys_status()) {
    if( E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_XY_CALIBRATING_PRINTING, &ret_sys_status)) {
      LOG_E("job_ctrl: Can not enter to SYSTEM_STATUS_XY_CALIBRATING_PRINTING status\r\n");
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    }
    else{
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_XY_CALIBRATING_PRINTING);
    }
  }
  else {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PRINTING, &ret_sys_status)) {
      LOG_E("job_ctrl: Can not enter SYS_PRINTING status\r\n");
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      return;
    }
    LOG_I("job_ctrl: enter SYS_PRINTING status\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_PRINTING);
  }
}

void JobCtrl::do_pause(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSING, &ret_sys_status)) {
    LOG_E("job ctrl: can not to enter SYS_PAUSEING status\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_PAUSING);

  switch (jri.req_data.req_pause_data.type) {
    case PAUSE_CLIENT_REQ:
      // motion_svc.normalstop();
      motion_svc.req_quickstop();
    break;

    case PAUSE_FILM_RUNOUT:
      // motion_svc.normalstop();
      motion_svc.req_quickstop();
    break;

    case PAUSE_POWR_LOSE:
      LOG_I("TODO: quickstop\r\n");
    break;

    case PAUSE_DOOR_OPEN:
      LOG_I("TODO: quickstop\r\n");
    break;

    case PAUSE_EXCEPTION:
      LOG_I("TODO: quickstop\r\n");
    break;

    default:
      LOG_E("job_ctrl: unknow pause type\r\n");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      return;
    break;
  }
  LOG_I("%d after quickstop\r\n", millis());

  if (E_SUCCESS != save_env()) {
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }

  if (E_SUCCESS != machine_standby()) {
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSED, &ret_sys_status)) {
    LOG_E("job ctrl: can not enter SYS_PAUSED status");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }

  _paused = true;
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_PAUSED);
}

void JobCtrl::do_resume(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_RESUMING, &ret_sys_status)) {
    LOG_E("job_ctrl: Can not enter to SYS_RESUMING status\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_RESUMING);

  if (E_SUCCESS != resum_env()) {
    LOG_E("job ctrl: resume failed\r\n");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_IDLE);
    return;
  }
  _client_id = jri.req_data.req_resume_data.client_id;
  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  _gcode_rb.reset();
  UNLOCK(_lock);

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PRINTING, NULL)) {
    LOG_E("job ctrl: can not enter SYS_PRINTING status");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_PRINTING);
}

void JobCtrl::do_stop(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (SYSTEM_STATUS_XY_CALIBRATING_PRINTING == smprinter.get_sys_status()) {
    // TODO: do nothing
  }
  else {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_STOPING, &ret_sys_status)) {
      LOG_E("job_ctrl: Can not enter to SYSTEM_STATUS_STOPING status\r\n");
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      return;
    }
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_STOPING);
  }

  // motion_svc.normalstop();
  motion_svc.req_quickstop();
  if (E_SUCCESS != machine_standby()) {
    LOG_E("job ctrl: machine standby failure\r\n");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    return;
  }

  if (SYSTEM_STATUS_XY_CALIBRATING_PRINTING == smprinter.get_sys_status()) {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_XY_CALIBRATING, &ret_sys_status)) {
      LOG_E("job ctrl: can not enter SYSTEM_STATUS_XY_CALIBRATING status");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
    }
    else {
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_XY_CALIBRATING);
    }
  }
  else {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status)) {
      LOG_E("job ctrl: can not enter SYS_IDLE status");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      return;
    }
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, SYSTEM_STATUS_IDLE);
  }
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
  ModuleBase *cur_toolhead;
  uint32_t cmd_len;

  if (SYSTEM_STATUS_PRINTING != smprinter.get_sys_status() && 
      SYSTEM_STATUS_FINISHING != smprinter.get_sys_status()) {
    return false;
  }

  // TODO: check the emergency event here
  // If need to stop quickly, now do it

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
      *line = _env.cur_line_num++;
      cmd[cmd_len] = 0;
      LOG_I("job_ctrl: marlin consume a gcode: %s\r\n", cmd);
      ret = true;

      if (_paused){
        if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
          // LOG_E("job_ctrl: can NOT get toolhead\r\n");
          _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION);
          req_stop();
          ret = false;
          break;
        }
        if(E_SUCCESS != cur_toolhead->resume_finish()) {
          _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION);
          req_stop();
          ret = false;
          break;
        }
        _paused = false;
      }
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
