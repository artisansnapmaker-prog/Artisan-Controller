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
#include "motion_platform.h"
#include "job_ctrl.h"
#include "emergency_handler.h"


static AT_CCRAM StackType_t stack_jobctrl_thread[SYSTEM_TASK_STACK_SIZE];
static AT_CCRAM StaticTask_t tcb_jobctrl;

static AT_CCRAM uint8_t gcode_ring_buffer[GCODE_RB_SIZE];
static AT_CCRAM uint8_t issue_ret_rb[4];

static AT_CCRAM uint8_t queue_buffer_jobctrl[JOB_CTRL_REQ_INFO_BUF];
static AT_CCRAM StaticMessageBuffer_t queue_strcut_jobctrl;

JobCtrl AT_CCRAM job_ctrl_svc;


void job_ctrl_thread_entry(void *p) {
  for(;;) {
    job_ctrl_svc.background_thread(p);
  }
}

void JobCtrl::init(void) {
  _lock = xSemaphoreCreateMutex();
  configASSERT(_lock);

  _req_queue = xMessageBufferCreateStatic(JOB_CTRL_REQ_INFO_BUF, queue_buffer_jobctrl, &queue_strcut_jobctrl);
  configASSERT(_req_queue);

  _gcode_rb.init(gcode_ring_buffer, (int32_t)GCODE_RB_SIZE);

  _issue_ret_rb.init(issue_ret_rb, 4);
  _statistics_log_interval_ms = 0;
  _statistics_log_last_tick_ms = 0;
  _env.gfi_valid = false;
  abort_resume = false;
  status_before_start = SYSTEM_STATUS_IDLE;
  got_last_gcode_packet = false;

  TaskHandle_t jobctrl_task = xTaskCreateStatic((TaskFunction_t)(job_ctrl_thread_entry), "jobctrl", SYSTEM_TASK_STACK_SIZE,
        (void *)(this), HIGHEST_TASK_PRIORITY,  stack_jobctrl_thread, &tcb_jobctrl);
  if (!jobctrl_task) {
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
  if (!got_last_gcode_packet && smprinter.on_printing()) {
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

  if (got_last_gcode_packet && _gcode_rb.is_empty() ){
    got_last_gcode_packet = false;
    LOG_I("push all gcodes to marlin or other 3D printer\r\n");
    req_stop(STOP_NORMAL, E_JOB_ISSUE_RET_FINISH);
  }

  uint8_t issue_ret;
  while (_issue_ret_rb.available()) {
    issue_ret = 0;
    _issue_ret_rb.remove_one(issue_ret);
    issue_nodify(issue_ret);
  }
}

err_code_t JobCtrl::req_start(  uint8_t client_id,
                                struct GcodeFileInfo *gcodeInfo,
                                toolHeadType th_type,
                                job_req_notify_cb_t cb/* = NULL*/,
                                void *p/* = NULL*/) {
  err_code_t ret = smprinter.can_start_work();
  if (ret != E_SUCCESS) {
    LOG_E("req_start: can not start job, ret=[%u]\r\n", ret);
    return ret;
  }

  status_before_start = smprinter.get_sys_status();
  LOG_I("Start a printing job at status: %u\r\n", status_before_start);

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
  if (  SYSTEM_STATUS_PRINTING != smprinter.get_sys_status() &&
        SYSTEM_STATUS_XY_CALIBRATING_PRINTING != smprinter.get_sys_status() &&
        SYSTEM_STATUS_RESUMING != smprinter.get_sys_status()) {
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
                                void *p/* = NULL*/,
                                JobResumeType pt/* = RESUME_TYPE_PAUSE*/) {
  err_code_t ret = smprinter.can_resume_work();
  if (ret != E_SUCCESS) {
    LOG_E("job_ctrl: Can not resume a job\r\n");
    return ret;
  }

  JobCtrlReqInfo jri;
  jri.req_action = REQ_RESUME;
  jri.req_data.req_resume_data.client_id = client_id;
  jri.req_data.req_resume_data.type = pt;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::req_stop( enum JobStopType st,
                              uint8_t reason,
                              job_req_notify_cb_t cb/* = NULL*/,
                              void *p/* = NULL*/) {
  if (!smprinter.can_stop_work()) {
    LOG_E("job_ctrl: Can not stop a job as current status is no working or paused\r\n");
    return E_JOB_NOT_IN_PAUSE_STATUS;
  }

  abort_resume = true;

  JobCtrlReqInfo jri;
  jri.req_action = REQ_STOP;
  jri.req_data.req_stop_data.type = st;
  jri.req_data.req_stop_data.reason = reason;
  jri.cb = cb;
  jri.param = p;

  if (sizeof(jri) != xMessageBufferSend(_req_queue, &jri, sizeof(jri), pdMS_TO_TICKS(100))) {
    LOG_E("job_ctrl: can not submit a job ctrl request\r\n");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}

void JobCtrl::print_job_env(struct JobEnv *env) {
  LOG_I("job_ctrl: ========================= save env =========================\r\n");
  LOG_I("TYPE: %d\r\n", env->type);
  LOG_I("active_coordinate: %d\r\n", env->active_coordinate);
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

  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: Can not get toolhead\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }

  _env.toolhead_env_buf_size = MODULE_ENV_MAX_SIZE;
  if (E_SUCCESS != cur_toolhead->save_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("Toolhead save env error\r\n");
    return E_JOB_SAVE_ENV_FAILURE;
  }

  if (TH_TYPE_3DP == _env.type){
    ModuleBase *bed;
    bed = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (bed) {
      _env.bed_env_buf_size = MODULE_ENV_MAX_SIZE;
      if (E_SUCCESS != bed->save_env(_env.bed_env_buf, _env.bed_env_buf_size)) {
        LOG_E("job_ctrl: bed save env failure\r\n");
      }
      else {
        // save_env() maybe called from ISR, so comment the log
        // LOG_I("job_ctrl: bed_temp save\r\n");
      }
    }
    else {
      LOG_E("job_ctrl: can not get bed\r\n");
    }
  }
  _env.active_coordinate = motion_platform_svc.get_active_coordinate_system();
  _env.cur_line_num = smprinter.gcode_file_position;
  _env.print_feadrate = motion_platform_svc.get_feedrate();
  _env.travel_feadrate = motion_platform_svc.get_travl_feedrate();
  _env.g0g1_relative_mode = motion_platform_svc.get_relative_mode();
  motion_platform_svc.update_position_from_platform();
  _env.current_pos = motion_platform_svc.sm_current_position;
  _env.E_stepper_count = motion_platform_svc.get_stepper_count(E_AXIS);

  for (int i = 0; i < BED_ZONE_MAX; i++) {
    _env.bed_temp[i] = motion_platform_svc.get_bed_temp(i);
  }

  // LOG_I("job_ctrl: save cur_line_num %d\r\n", _env.cur_line_num);
  // print_job_env(&_env);
  return E_SUCCESS;
}

err_code_t JobCtrl::recover_env(void) {
  ModuleBase *cur_toolhead;

  LOG_I("job_ctrl: get current toolhead pointer in recover_env\r\n");
  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: can not get toolhead in recover_env\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  // Check toolhead
  LOG_I("job_ctrl: check current toolhead type in recover_env\r\n");
  if (smprinter.get_toolhead_type() != _env.type) {
    return E_JOB_UNSUPPORT_PARAM;
  }

  if (smprinter.get_toolhead_type() == TH_TYPE_3DP) {
    for (int i = 0; i < BED_ZONE_MAX; i++) {
      LOG_I("job_ctrl: recover zone[%d] temp to %d\r\n", i, _env.bed_temp[i]);
      motion_platform_svc.set_bed_temp(_env.bed_temp[i], i);
    }
  }

  if (E_SUCCESS != cur_toolhead->recover_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("job_ctrl: can not resume toolhead in recover_env\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t JobCtrl::resume_env(JobResumeType rt) {
  ModuleBase *cur_toolhead;
  xyze_pos_t dest;

  if (rt == RESUME_TYPE_LIVE_Z_OFFSET) {
    goto __pos_resume;
  }

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

  if (E_SUCCESS != cur_toolhead->resume_env(_env.toolhead_env_buf, _env.toolhead_env_buf_size)) {
    LOG_E("job_ctrl: can not resume toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

  if (rt != RESUME_TYPE_LIVE_Z_OFFSET && TH_TYPE_3DP == _env.type) {
    while(!abort_resume &&
          (!motion_platform_svc.bed_heatup_to_target() ||
          !motion_platform_svc.hotends_heatup_to_target())) {
      LOG_I("job_ctrl: wait for bed and hotends heatup to target\r\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  abort_resume = false;

__pos_resume:
  _env.req_line_num = _env.cur_line_num;
  // wait for all movement done
  motion_platform_svc.synchronize_planner();
  motion_platform_svc.update_position_from_platform();

  dest   = motion_platform_svc.sm_current_position;
  dest.x = _env.current_pos.x;
  dest.y = _env.current_pos.y;
  motion_platform_svc.moveto(dest, RESUME_XY_FEEDRATE);

  dest.z = _env.current_pos.z;
  motion_platform_svc.moveto(dest, RESUME_Z_FEEDRATE);

  motion_platform_svc.set_feedrate(_env.print_feadrate);
  motion_platform_svc.set_travl_feedrate(_env.travel_feadrate);
  motion_platform_svc.set_relative_mode(_env.g0g1_relative_mode);

  // wait for all movement done
  motion_platform_svc.synchronize_planner();

  // set position to planner for E, I, J
  motion_platform_svc.update_position_from_platform();
  motion_platform_svc.sm_current_position.e = _env.current_pos.e;
  motion_platform_svc.sm_current_position.i = _env.current_pos.i;
  motion_platform_svc.sm_current_position.j = _env.current_pos.j;
  motion_platform_svc.sync_plan_position_to_platform();

  // motion_platform_svc.set_stepper_count(E_AXIS, _env.E_stepper_count);
  // LOG_I("job_ctrl: resume cur_line_num %d\r\n", _env.cur_line_num);
  // LOG_I("job_ctrl: ========================= resume =========================\r\n");
  // LOG_I("job_ctrl: resume active coordinate_system %d\r\n", motion_platform_svc.get_active_coordinate_system());
  // LOG_I("job_ctrl: resume req_line_num %d\r\n", _env.req_line_num);
  // LOG_I("job_ctrl: resume relative mode %d\r\n", _env.g0g1_relative_mode);
  // LOG_I("job_ctrl: resume xy position %f %f\r\n", _env.current_pos[0], _env.current_pos[1]);
  // LOG_I("job_ctrl: resume z position %f\r\n", _env.current_pos[2]);
  // LOG_I("job_ctrl: resume print_feadrate %f\r\n", _env.print_feadrate);
  // LOG_I("job_ctrl: resume travel_feadrate %f\r\n", _env.travel_feadrate);
  // LOG_I("job_ctrl: resume relative mode %d\r\n", _env.g0g1_relative_mode);
  // LOG_I("\r\n");

  return E_SUCCESS;
}

err_code_t JobCtrl::machine_standby(void) {
  ModuleBase *cur_toolhead;
  xyze_pos_t t_pos;

  LOG_I("%d job_ctrl: machine standby begin\r\n", millis());
  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    LOG_E("job_ctrl: can not get toolhead\r\n");
    return E_JOB_RESUME_ENV_FAILURE;
  }

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
    motion_platform_svc.update_position_from_platform();
    t_pos =  motion_platform_svc.sm_current_position;
    t_pos.e -= 5;
    motion_platform_svc.moveto(t_pos, 10, true);
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

  if (TH_TYPE_3DP == _env.type || TH_TYPE_CNC == _env.type) {
    LOG_I("job_ctrl: Z raise to highest\r\n");
    motion_platform_svc.update_position_from_platform();
    t_pos = motion_platform_svc.sm_current_position;
    t_pos.z = 395;
    motion_platform_svc.moveto(t_pos, RESUME_Z_FEEDRATE, true);

    LOG_I("job_ctrl: y move to fronthead\r\n");
    motion_platform_svc.update_position_from_platform();
    t_pos = motion_platform_svc.sm_current_position;
    t_pos.y = 395;
    motion_platform_svc.moveto(t_pos, RESUME_XY_FEEDRATE, true);
  }

  LOG_I("job_ctrl: machine standby end\r\n");
  return E_SUCCESS;
}

void JobCtrl::get_gcodes_from_client(void) {
  req_batch_gcode_t req_batch_gcode;
  res_batch_gcode_t res_batch_gcode;
  uint8_t batch_gcode_buf[GCODE_REQ_BUFFER_MIN];

  // while((uint32_t)_gcode_rb.free() >= _get_gcode_buffer_req_min) {
  while((uint32_t)_gcode_rb.free() >= GCODE_REQ_BUFFER_MIN) {
    req_batch_gcode.line_num = _env.req_line_num;
    req_batch_gcode.buf_len = (uint16_t) (MIN((uint32_t)_gcode_rb.free(), GCODE_REQ_BUFFER_MIN));
    //if (req_batch_gcode.buf_len < _get_gcode_buffer_req_min){
      // LOG_I("job_ctrl: no large enough buffer for get gcode, minimum request %d, but we juest get %d\r\n", _get_gcode_buffer_req_min, req_batch_gcode.buf_len);
      //break;
    //}
    res_batch_gcode.gcode_str = batch_gcode_buf;
    // LOG_I("job_ctrl: get gcode from client %d, startline %d, buffer %d\r\n", _client_id, req_batch_gcode.line_num, req_batch_gcode.buf_len);
    if(ClientNode::get_batch_gcode(_client_id, req_batch_gcode, res_batch_gcode)) {
      if (E_SUCCESS != res_batch_gcode.result &&
          E_JOB_LAST_GCODE_PACK != res_batch_gcode.result) {
        LOG_E("job_ctrl: get gcode's result error\r\n");
        _err_get_batch_gcode_cnt++;
        continue;
      }

      if(res_batch_gcode.start_line_num != req_batch_gcode.line_num) {
        LOG_E("start line number not match, drop this batch gcode\r\n");
        _err_get_batch_gcode_cnt++;
        // req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER);
        break;
      }
      // shoule we check the line number?
      uint8_t *p, *ls;
      p = ls = res_batch_gcode.gcode_str;
      LOG_I("SART: %d ~ STOP: %d\r\n", res_batch_gcode.start_line_num, res_batch_gcode.end_line_num);
      // LOG_I("get gcode:\r\n %s\r\n", p);
      // uint8_t str_temp[MAX_CMD_SIZE];
      uint32_t rx_line_num = 0;
      {
        while('\0' != *p) {
          if ('\n' == *p) {
            rx_line_num++;
            // for debug
            // if (p - ls < MAX_CMD_SIZE) {
            //   memcpy(str_temp, ls, (p - ls));
            //   str_temp[p-ls] = 0;
            //   ls = p + 1;
            //   LOG_I("job_ctrl: get gocde: %s\r\n", (char *)str_temp);
            // }
          }
          p++;
        }

        if (0 == rx_line_num) {
          // LOG_I("job_ctrl: get 0 line gcode, perhaps no large buffer for the next gcode, break and wait for the next large gcode buffer\r\n");
          // update
          _get_gcode_buffer_req_min = req_batch_gcode.buf_len + 1;
        }

        if (rx_line_num != ((res_batch_gcode.end_line_num - res_batch_gcode.start_line_num) + 1)) {
          LOG_E("line number not match, drop this batch gcode, expect %d, but get %d\r\n", rx_line_num, ((res_batch_gcode.end_line_num - res_batch_gcode.start_line_num) + 1));
          // req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER);
          _err_get_batch_gcode_cnt++;
          break;
        }

        // gcode ringbuffer guarantee to hold all the gcode string.
        _gcode_rb.insert_multi(res_batch_gcode.gcode_str, p - res_batch_gcode.gcode_str);
        _env.req_line_num += rx_line_num;
        _err_get_batch_gcode_cnt = 0;

        if (E_JOB_LAST_GCODE_PACK == res_batch_gcode.result) {
          LOG_I("job_ctrl: Job control get last gcode packe, last gcode line number %d\r\n", _env.req_line_num - 1);
          got_last_gcode_packet = true;
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
    LOG_W("can not get batch gcode from clinet for 3 times\r\n");
    _err_get_batch_gcode_cnt = 0;
    // req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER);
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

void JobCtrl::stepper_quickstop_cb(void) {
  ModuleBase *cur_toolhead;

  // Call from stepper ISR
  SystemStatus s = smprinter.get_sys_status();
  if (SYSTEM_STATUS_PAUSING != s &&
      SYSTEM_STATUS_STOPING != s &&
      SYSTEM_STATUS_XY_CALIBRATING_PRINTING != s) {
    return;
  }

  // Just do it for laster quickstop
  if (TH_TYPE_LASER != smprinter.get_toolhead_type()) {
    return;
  }

  if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
    return;
  }

  cur_toolhead->quickstop();
}

void JobCtrl::update_gcode_file_pass_line_number(uint32_t l) {
  if (l >= (_env.req_line_num - 1)) {
    last_gcode_execute_by_platform = true;
  }
}

void JobCtrl::do_start(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;
  enum SystemStatus next_status;

  err_code_t ret = smprinter.can_start_work();

  if (ret != E_SUCCESS) {
    LOG_E("can not start job as current status is not idle or calibrating\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret);
    return;
  }

  // if start work from idle:
  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_STARTING, &ret_sys_status)) {
    LOG_E("job_ctrl: Can not enter to SYS_STARTING at status: %u\r\n", ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
    return;
  }

  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);

  // prepare flash to record emergency data
  emergency_hdl.prepare_flash();

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
  got_last_gcode_packet = false;
  _paused = false;
  motion_platform_svc.stepper_total_offset = 0;

  // get next status we should enter
  switch (status_before_start) {
  case SYSTEM_STATUS_XY_CALIBRATING:
    next_status = SYSTEM_STATUS_XY_CALIBRATING_PRINTING;
    break;

  case SYSTEM_STATUS_LASER_CAMERA_CAPTURE:
  case SYSTEM_STATUS_LASER_DETECT_FOCAL_LENGTH:
  case SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION:
    next_status = SYSTEM_STATUS_LASER_CALIBRATION_PRINTING;
    break;

  default:
    next_status = SYSTEM_STATUS_PRINTING;
    break;
  }

  // requst enter next status
  if( E_SUCCESS != smprinter.set_sys_status(next_status, &ret_sys_status)) {
    LOG_E("job_ctrl: Can not enter to printing mode[%u] at current status[%u]\r\n", next_status, ret_sys_status);
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
  }
  else{
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);
    for (int i = 0; i < JOB_CTRL_NOTIFY_QUEUE_SIZE; i++) {
      if (notify_handle_started[i].cb) {
        // tell object the status we start working from
        notify_handle_started[i].cb(notify_handle_started[i].obj, status_before_start);
      }
    }
  }
}

void JobCtrl::do_pause(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;
  uint32_t start_millis, end_millis;
  bool need_standby = false;

  ret_sys_status = smprinter.get_sys_status();
  if (  SYSTEM_STATUS_PRINTING != ret_sys_status &&
        SYSTEM_STATUS_XY_CALIBRATING_PRINTING != ret_sys_status &&
        SYSTEM_STATUS_RESUMING != ret_sys_status) {

    LOG_E("job client: can not pause a job as current status[%u] is no printing\r\n", ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_NOT_IN_WORKING_STATUS);
    return;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSING, &ret_sys_status)) {
    LOG_E("job ctrl: can not to enter SYS_PAUSEING status\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
    return;
  }

  start_millis = millis();
  switch (jri.req_data.req_pause_data.type) {

    case PAUSE_POWER_LOSE:
    break;

    case PAUSE_CLIENT_REQ:
    case PAUSE_FILM_RUNOUT:
    case PAUSE_DOOR_OPEN:
    case PAUSE_EXCEPTION:
    case PAUSE_WRONG_EXTRUDER:
    case PAUSE_WRONG_NOZZLE:
    case PAUSE_NOZZLE_TEMP:
      motion_platform_svc.req_quickstop();
      need_standby = true;
    break;

    case PUASE_LIVE_Z_OFFSET:
      motion_platform_svc.req_live_Z_offset_quickstop();
      need_standby = false;
    break;

    default:
      LOG_E("job_ctrl: unknow pause type %d\r\n", jri.req_data.req_pause_data.type);
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, ret_sys_status);
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_PAUSE_PARAM_ERR);
      return;
    break;
  }

  if (E_SUCCESS != save_env()) {
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_SAVE_ENV_FAILURE);
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_SAVE_ENV_FAILURE);
    return;
  }

  // TODO: tell hmi the result firstly
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);

  end_millis = millis();
  LOG_I("quick stop to standby take %d milliseconds\r\n",
      end_millis >= start_millis?
      end_millis - start_millis :
      ((int)end_millis - (int)start_millis));

  if (need_standby && E_SUCCESS != machine_standby()) {
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_STANDBY_FAILED);
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_FAILURE);
    return;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PAUSED, &ret_sys_status)) {
    LOG_E("job ctrl: can not enter SYS_PAUSED status");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
    return;
  }

  _paused = true;
  if (PAUSE_DOOR_OPEN == jri.req_data.req_pause_data.type) {
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_DOOR_OPEN);
  } else if (PAUSE_WRONG_EXTRUDER == jri.req_data.req_pause_data.type) {
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_WRONG_EXTRUDER);
  } else if (PAUSE_WRONG_NOZZLE == jri.req_data.req_pause_data.type) {
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_WRONG_NOZZLE);
  } else if (PAUSE_NOZZLE_TEMP == jri.req_data.req_pause_data.type) {
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_WRONG_HOTEND_TEMP);
  } else if (PAUSE_EXCEPTION == jri.req_data.req_pause_data.type) {
    if (smprinter.get_toolhead_type() == TH_TYPE_CNC)
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION);
  } else if (PAUSE_FILM_RUNOUT == jri.req_data.req_pause_data.type) {
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_FILAMENT_RUNOUT);
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);

  // notify objects we have paused working
  for (int i = 0; i < JOB_CTRL_NOTIFY_QUEUE_SIZE; i++) {
    if (notify_handle_paused[i].cb) {
      // tell objects the type of pause source
      notify_handle_paused[i].cb(notify_handle_paused[i].obj, jri.req_data.req_pause_data.type);
    }
  }
}

void JobCtrl::do_resume(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_RESUMING, &ret_sys_status)) {
    LOG_E("job_ctrl: Can not enter to SYS_RESUMING status\r\n");
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
    return;
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);

  // to make job ctrl call resume_finish()
  if (jri.req_data.req_resume_data.type == RESUME_TYPE_RECOVERY) {
    if (E_SUCCESS != recover_env()) {
      LOG_E("job ctrl: recover env failed\r\n");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_RECOVER_ENV_FAILED);
      return;
    }
  }

  if (E_SUCCESS != resume_env(jri.req_data.req_resume_data.type)) {
    LOG_E("job ctrl: resume env failed\r\n");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_RESUME_ENV_FAILURE);
    return;
  }

  if (RESUME_TYPE_LIVE_Z_OFFSET != jri.req_data.req_resume_data.type) {
    _client_id = jri.req_data.req_resume_data.client_id;
  }

  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  _gcode_rb.reset();
  UNLOCK(_lock);
  got_last_gcode_packet = false;
  motion_platform_svc.stepper_total_offset = 0;

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_PRINTING, NULL)) {
    LOG_E("job ctrl: can not enter SYS_PRINTING status");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_PAUSE_FAILURE);
    return;
  }
  DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);

  // to here, it indicates recovery is done, by scott
  if (jri.req_data.req_resume_data.type == RESUME_TYPE_RECOVERY) {
    // set this flash to make jobctrl thinks it recovery from paused
    _paused = true;
    // prepare flash for possible emergency event
    emergency_hdl.prepare_flash();
    // TODO: we should tell HMI the result after all ok
  }

  // notify objects we will resume working
  for (int i = 0; i < JOB_CTRL_NOTIFY_QUEUE_SIZE; i++) {
    if (notify_handle_resume[i].cb) {
      // tell objects the type of resume source
      notify_handle_resume[i].cb(notify_handle_resume[i].obj, jri.req_data.req_resume_data.type);
    }
  }
}

void JobCtrl::do_stop(struct JobCtrlReqInfo &jri) {
  enum SystemStatus ret_sys_status;

  if (SYSTEM_STATUS_PRINTING == smprinter.get_sys_status()) {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_STOPING, &ret_sys_status)) {
      LOG_E("job_ctrl: Can not enter to SYSTEM_STATUS_STOPING status\r\n");
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_FAILURE);
      return;
    }
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);
  }
  else {
    // TODO: do nothing
  }

  LOCK(_lock, JOB_LOCK_WAIT_TICK);
  _gcode_rb.reset();
  UNLOCK(_lock);

  switch(jri.req_data.req_stop_data.type) {
    case STOP_NORMAL:
    {
      uint32_t cnt = 30;
      while (motion_platform_svc.planner_busy() || !last_gcode_execute_by_platform) {
        vTaskDelay(10);
        if (0 == cnt % 30)
          LOG_I("gcode_file_pass_line_number %d\r\n", smprinter.gcode_file_pass_line_number);
        cnt++;
      }
    }
    break;

    case STOP_CLIENT_REQ:
      // if homing now, use normalstop
      // else use quickstop
      while(motion_platform_svc.homing_now) vTaskDelay(5);
      motion_platform_svc.req_quickstop();
    break;

    case STOP_EXCEPTION:
      motion_platform_svc.req_quickstop();
      smprinter.set_sys_status(SYSTEM_STATUS_EMERGENCY_STOP, NULL);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);
      _issue_ret_rb.insert_one(jri.req_data.req_stop_data.reason);
      return;

    case STOP_EMERGENCY:
      motion_platform_svc.req_quickstop();
    break;

    default:
      LOG_E("Unknow stop type");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_PARAM_ERR);
      return;
    break;
  }

  // TODO: emergency

  if (E_SUCCESS != machine_standby()) {
    LOG_E("job ctrl: machine standby failure\r\n");
    smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_JOB_STANDBY_FAILED);
    _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_FAILURE);
    return;
  }

  // normal printing
  if (SYSTEM_STATUS_PRINTING == smprinter.get_sys_status()) {
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status)) {
      LOG_E("job ctrl: can not enter SYS_IDLE status");
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_FAILURE);
      return;
    }

    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);
    _issue_ret_rb.insert_one(jri.req_data.req_stop_data.reason);
    return;
  }

  // other printing
  {
    if (E_SUCCESS != smprinter.set_sys_status(status_before_start, &ret_sys_status)) {
      LOG_E("job ctrl: can not enter status: %u\n", status_before_start);
      smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_sys_status);
      _issue_ret_rb.insert_one(SACP_JOB_PAUSE_ISSUE_RET_STOP_FAILURE);
      DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_FAILURE);
      return;
    }

    DO_JOB_REQ_NOTIFY_CB(jri.cb, jri.param, E_SUCCESS);
    _issue_ret_rb.insert_one(jri.req_data.req_stop_data.reason);
    // reset the status
    status_before_start = SYSTEM_STATUS_IDLE;
  }

  for (int i = 0; i < JOB_CTRL_NOTIFY_QUEUE_SIZE; i++) {
    if (notify_handle_stopped[i].cb) {
      // tell objects the type of stop source
      notify_handle_stopped[i].cb(notify_handle_stopped[i].obj, jri.req_data.req_stop_data.type);
    }
  }
}

err_code_t JobCtrl::set_env(struct JobEnv &env) {
  // TODO: check env
  _env = env;
  return E_SUCCESS;
}

struct JobEnv *JobCtrl::get_env(void) {
  return &_env;
}

err_code_t JobCtrl::update_env(void) {
  return save_env();
}

bool JobCtrl::consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  bool ret;
  uint8_t c;
  ModuleBase *cur_toolhead;
  uint32_t cmd_len;

  if (!smprinter.on_printing()) {
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
      *line = _env.cur_line_num++;
      cmd[cmd_len] = 0;
      // LOG_I("job_ctrl: marlin consume a gcode %d: %s\r\n", *line, cmd);
      last_gcode_execute_by_platform = false;
      ret = true;

      if (_paused){
        if (!(cur_toolhead = smprinter.get_cur_toolhead())) {
          // LOG_E("job_ctrl: can NOT get toolhead\r\n");
          req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION);
          ret = false;
          break;
        }
        if(E_SUCCESS != cur_toolhead->resume_finish()) {
          req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION);
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

err_code_t JobCtrl::register_notify_handle(JobNotifyType type, void *obj, job_req_notify_cb_t cb) {
  JobCtrlNotifyHandle *handles = NULL;
  int i = 0;

  switch (type) {
  case JOB_NOTIFY_TYPE_STARTED:
    handles = notify_handle_started;
    break;

  case JOB_NOTIFY_TYPE_PAUSED:
    handles = notify_handle_paused;
    break;

  case JOB_NOTIFY_TYPE_RESUME:
    handles = notify_handle_resume;
    break;

  case JOB_NOTIFY_TYPE_STOPPED:
    handles = notify_handle_stopped;
    break;

  default:
    break;
  }

  if (!handles) {
    LOG_E("failed to register notify handle, error type:[%u]\n", type);
    return E_PARAM;
  }

  for (; i < JOB_CTRL_NOTIFY_QUEUE_SIZE; i++) {
    if ((NULL == handles[i].cb && NULL == handles[i].obj) || handles[i].obj == obj) {
      handles[i].obj = obj;
      handles[i].cb  = cb;
      break;
    }
  }

  if (i >= JOB_CTRL_NOTIFY_QUEUE_SIZE) {
    LOG_E("failed to register notify handle for type:[%u]\n", type);
    return E_NO_RESRC;
  }

  LOG_I("has register notify handle for type:[%u]\n", type);

  return E_SUCCESS;
}
