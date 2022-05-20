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
#ifndef SNAPMAKER_JOB_CTRL_SERVICE_H_
#define SNAPMAKER_JOB_CTRL_SERVICE_H_


#include <functional>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "../common/list.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
// #include "../snapmaker.h"
#include "client_node.h"
#include "motion_platform.h"


#define E_JOB_FAILURE                                   SACP_RET_FAILURE
#define E_JOB_LAST_GCODE_PACK                           SACP_RET_JOB_LAST_GCODE_PACK
#define E_JOB_NOT_IN_IDLE_STATUS                        SACP_RET_JOB_NOT_IN_IDLE_STATUS
#define E_JOB_NO_HOME                                   SACP_RET_JOB_NO_HOME
#define E_JOB_IVALID_GCODE_FILE                         SACP_RET_JOB_IVALID_GCODE_FILE
#define E_JOB_NOT_IN_WORKING_STATUS                     SACP_RET_JOB_NOT_IN_WORKING_STATUS
#define E_JOB_NOT_IN_PAUSE_STATUS                       SACP_RET_JOB_NOT_IN_PAUSE_STATUS
#define E_JOB_IVALID_POWER_LOSE_DATA                    SACP_RET_JOB_IVALID_POWER_LOSE_DATA
#define E_JOB_POWER_LOSE_CHECK_FAILURE                  SACP_RET_JOB_POWER_LOSE_CHECK_FAILURE
#define E_JOB_GCODE_FILE_NO_EXIT                        SACP_RET_JOB_GCODE_FILE_NO_EXIT
#define E_JOB_SAVE_ENV_FAILURE                          SACP_RET_JOB_SAVE_ENV_FAILURE
#define E_JOB_RESUME_ENV_FAILURE                        SACP_RET_JOB_RESUME_ENV_FAILURE
#define E_JOB_UNSUPPORT_PARAM                           SACP_RET_UNSUPPORT_PARAM
#define E_JOB_UNMATCHED_TOOLHEAD                        SACP_RET_JOB_UNMATCHED_TOOLHEAD
#define E_JOB_NO_TOOLHEAD                               SACP_RET_JOB_NO_TOOLHEAD
#define E_JOB_TOOLHEAD_OFFLINE                          SACP_RET_JOB_TOOLHEAD_OFFLINE
#define E_JOB_EXCEPTION_BAN                             SACP_RET_JOB_EXCEPTION_BAN
#define E_JOB_FDM_EXTRUDER_STATE                        SACP_RET_JOB_FDM_EXTRUDER_STATE
#define E_JOB_FDM_NOZZLE_TYPE                           SACP_RET_JOB_FDM_NOZZLE_TYPE
#define E_JOB_FDM_NOZZLE_TEMP                           SACP_RET_JOB_FDM_NOZZLE_TEMP
#define E_JOB_FDM_FILAMENT_RUNOUT                       SACP_RET_JOB_FDM_FILAMENT_RUNOUT
#define E_JOB_CNC_OVERCURRENT                           SACP_RET_JOB_CNC_OVERCURRENT
#define E_JOB_CNC_P_TEMP_EXCE                           SACP_RET_JOB_CNC_P_TEMP_EXCE
#define E_JOB_CNC_M_TEMP_EXCE                           SACP_RET_JOB_CNC_M_TEMP_EXCE
#define E_JOB_CNC_V_POWER_EXC                           SACP_RET_JOB_CNC_V_POWER_EXC
#define E_JOB_ENCLOSURE_DOOR_OPEN                       SACP_RET_JOB_ENCLOSURE_DOOR_OPEN
#define E_JOB_LASER_IMU_CONNECTION                      SACP_RET_JOB_LASER_IMU_CONNECTION
#define E_JOB_LASER_TUBE_TEMP_TOO_HIGH                  SACP_RET_JOB_LASER_TUBE_TEMP_TOO_HIGH
#define E_JOB_LASER_ABNORMAL_ATTTUDE                    SACP_RET_JOB_LASER_ABNORMAL_ATTTUDE
#define E_JOB_LASER_INVLAID_PWN_PIN                     SACP_RET_JOB_LASER_INVLAID_PWN_PIN
#define E_JOB_LASER_TUBE_TEMP_TOO_LOW                   SACP_RET_JOB_LASER_TUBE_TEMP_TOO_LOW
#define E_JOB_LASER_IMU_OVERTEMP                        SACP_RET_JOB_LASER_IMU_OVERTEMP
#define E_JOB_RECOVER_ENV_FAILED                        SACP_RET_JOB_RECOVER_ENV_FAILED
#define E_JOB_STANDBY_FAILED                            SACP_RET_JOB_STANDBY_FAILED


#define E_JOB_ISSUE_RET_FINISH                          SACP_JOB_PAUSE_ISSUE_RET_FINISH
#define E_JOB_ISSUE_RET_GCODE_PAUSE                     SACP_JOB_PAUSE_ISSUE_RET_GCODE_PAUSE
#define E_JOB_ISSUE_RET_GCODE_FILAMENT_RUNOUT           SACP_JOB_PAUSE_ISSUE_RET_GCODE_FILAMENT_RUNOUT
#define E_JOB_ISSUE_RET_FILAMENT_RUNOUT                 SACP_JOB_PAUSE_ISSUE_RET_FILAMENT_RUNOUT
#define E_JOB_ISSUE_RET_STALL_PROTECTION                SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION
#define E_JOB_ISSUE_RET_ABNORMAL_TEMP_PROTECTION        SACP_JOB_PAUSE_ISSUE_RET_ABNORMAL_TEMP_PROTECTION
#define E_JOB_ISSUE_RET_IVALID_GCODE_LINE_NUMBER        SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER
#define E_JOB_ISSUE_RET_GET_GCODE_FAILURE               SACP_JOB_PAUSE_ISSUE_RET_GET_GCODE_FAILURE

#define JOB_LOCK_WAIT_TICK (0xFFFFFFFF)
#define MODULE_ENV_MAX_SIZE 128
#define GCODE_RB_SIZE 1024
#define RESUME_XY_FEEDRATE 50
#define RESUME_Z_FEEDRATE 30
#define JOB_CTRL_LOOP_TIME_MS (100)
#define JOB_CTRL_REQ_INFO_BUF ((sizeof(struct JobCtrlReqInfo) + 8) * 4)
#define DO_JOB_REQ_NOTIFY_CB(cb, p, ret)                do{ if(cb) (cb)(p, ret); } while(0)
#define GCODE_REQ_BUFFER_MIN 512

#define JOB_CTRL_NOTIFY_QUEUE_SIZE  (16)

enum JobPauseType {
  PAUSE_CLIENT_REQ,
  PAUSE_FILM_RUNOUT,
  PAUSE_POWER_LOSE,
  PAUSE_DOOR_OPEN,
  PAUSE_EXCEPTION,
  PAUSE_WRONG_EXTRUDER,
  PAUSE_WRONG_NOZZLE,
  PAUSE_NOZZLE_TEMP,
  PUASE_LIVE_Z_OFFSET,
};

enum JobResumeType {
  RESUME_TYPE_PAUSE,
  RESUME_TYPE_RECOVERY,
  RESUME_TYPE_LIVE_Z_OFFSET,
};

enum JobStopType {
  STOP_NORMAL,
  STOP_CLIENT_REQ,
  STOP_EMERGENCY,
  STOP_EXCEPTION,
};

enum JobNotifyType {
  JOB_NOTIFY_TYPE_STARTED,
  JOB_NOTIFY_TYPE_PAUSED,
  JOB_NOTIFY_TYPE_RESUME,
  JOB_NOTIFY_TYPE_STOPPED,
};


struct JobEnv {
  toolHeadType type;                                          /** job type :                                                            */
  bool gfi_valid;                                             /** gcode file information valid                                          */
  struct GcodeFileInfo gcode_file_info;                       /** gcode file information                                                */
  uint32_t req_line_num;                                      /** request line number                                                   */
  uint32_t cur_line_num;                                      /** current linenumber, use with gcode file to save the job point         */
  uint32_t time_elape;                                        /** time elaps from the job starting, in second                           */
  xyze_pos_t current_pos;
  uint32_t E_stepper_count;
  float print_feadrate;
  float travel_feadrate;
  bool g0g1_relative_mode;
  int16_t bed_temp[BED_ZONE_MAX];
  int8_t active_coordinate;
  uint32_t toolhead_env_buf_size;
  uint8_t toolhead_env_buf[MODULE_ENV_MAX_SIZE];
  uint32_t bed_env_buf_size;
  uint8_t bed_env_buf[MODULE_ENV_MAX_SIZE];
};

enum JobReqAction {
  REQ_START,
  REQ_PAUSE,
  REQ_RESUME,
  REQ_STOP,
};

// typedef void (*job_req_notify_cb)(uint8_t result);
// typedef std::function<void(int)> job_req_notify_cb_t;
typedef void(*job_req_notify_cb_t)(void *, uint8_t);

struct JobCtrlReqInfo {
  JobReqAction req_action;
  union
  {
    struct {
      uint8_t client_id;
      struct GcodeFileInfo gcodeInfo;
      toolHeadType th_type;
    } req_start_data;

    struct {
      enum JobPauseType type;
    } req_pause_data;

    struct {
      JobResumeType type;
      uint8_t client_id;
    } req_resume_data;

    struct {
      enum JobStopType type;
      uint8_t reason;
    } req_stop_data;
  } req_data;

  job_req_notify_cb_t cb;
  void *param;
};

struct JobCtrlNotifyHandle {
  void *obj;
  job_req_notify_cb_t cb;
};

class JobCtrl {
  // public methods
  public:
    JobCtrl() {
      // initialize notify handles
      memset(notify_handle_started, 0x00, sizeof(JobCtrlNotifyHandle) * JOB_CTRL_NOTIFY_QUEUE_SIZE);
      memset(notify_handle_paused, 0x00, sizeof(JobCtrlNotifyHandle) * JOB_CTRL_NOTIFY_QUEUE_SIZE);
      memset(notify_handle_resume, 0x00, sizeof(JobCtrlNotifyHandle) * JOB_CTRL_NOTIFY_QUEUE_SIZE);
      memset(notify_handle_stopped, 0x00, sizeof(JobCtrlNotifyHandle) * JOB_CTRL_NOTIFY_QUEUE_SIZE);
    }

    void init(void);
    void background_thread(void *p);                               /** main loop, to check all the event from system which will change current job status */

    // job control
    err_code_t req_start( uint8_t client_id,
                          struct GcodeFileInfo *gcodeInfo,
                          toolHeadType th_type,
                          job_req_notify_cb_t cb = NULL,
                          void *p = NULL);
    err_code_t req_pause( enum JobPauseType pt,
                          job_req_notify_cb_t cb = NULL,
                          void *p = NULL);
    err_code_t req_resume(uint8_t client_id,
                          job_req_notify_cb_t cb = NULL,
                          void *p = NULL,
                          JobResumeType pt = RESUME_TYPE_PAUSE);
    err_code_t req_stop(  enum JobStopType pt,
                          uint8_t reason,
                          job_req_notify_cb_t cb = NULL,
                          void *p = NULL);
    void print_job_env(struct JobEnv *env);

    // set & get
    err_code_t set_env(struct JobEnv &env);
    struct JobEnv *get_env(void);
    err_code_t update_env(void);
    toolHeadType get_type(void) { return _env.type; }
    struct GcodeFileInfo *get_gcode_info(void) { return _env.gfi_valid? &_env.gcode_file_info : NULL; }
    uint32_t get_cur_linenum(void) { return _env.cur_line_num; }
    uint32_t get_time_elaps(void) { return _env.time_elape; }
    void statistics_log_set(uint32_t interval_ms) { _statistics_log_interval_ms = interval_ms; };
    void statistics_output(void);
    void stepper_quickstop_cb(void);
    void update_gcode_file_pass_line_number(uint32_t l);

    // gcode
    bool consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line);
    bool gcode_file_info_check(struct GcodeFileInfo *gfi);

    err_code_t register_notify_handle(JobNotifyType type, void *obj, job_req_notify_cb_t);

  // private methods
  private:
    void do_start(struct JobCtrlReqInfo &jri);
    void do_pause(struct JobCtrlReqInfo &jri);
    void do_resume(struct JobCtrlReqInfo &jri);
    void do_stop(struct JobCtrlReqInfo &jri);
    err_code_t save_env(void);                                  /** save current job enviroment                                           */
    err_code_t resume_env(JobResumeType rt);                    /** resume saved enviroment to job                                        */
    err_code_t recover_env(void);                               /** resume saved enviroment to job                                        */
    err_code_t machine_standby(void);                           /** set the machine in standby status                                     */
    void issue_nodify(uint8_t issue_ret);
    void get_gcodes_from_client(void);

    SemaphoreHandle_t _lock;                                    /** lock, TODO:should use the snapmaker's API, not the freeRTOS           */
    MessageBufferHandle_t _req_queue;                           /** job control request enqueue this queue, the background thread outqueue requst and do it            */
    RingBuffer<uint8_t> _gcode_rb;                              /** ringbuffer for rx the gcode string                                    */
    uint8_t _client_id;                                         /** A pointer to client node,                                             */
    uint32_t _tick_ms;                                          /** use for periodically main loop                                        */
    struct JobEnv _env;                                         /** environment of this job, used to job resume                           */
    RingBuffer<uint8_t> _issue_ret_rb;                          /** ringbuffer for issue code                                             */
    bool _paused;
    SystemStatus status_before_start;
    bool got_last_gcode_packet;
    bool last_gcode_execute_by_platform;

    // use for state of self-inspection
    uint32_t _err_get_batch_gcode_cnt;
    uint32_t _statistics_log_interval_ms;
    uint32_t _statistics_log_last_tick_ms;

    // use for get gcode
    uint32_t _get_gcode_buffer_req_min;                       /** the minimum buffer use to get gcode                                       */
    bool abort_resume;

    JobCtrlNotifyHandle notify_handle_started[JOB_CTRL_NOTIFY_QUEUE_SIZE];
    JobCtrlNotifyHandle notify_handle_paused[JOB_CTRL_NOTIFY_QUEUE_SIZE];
    JobCtrlNotifyHandle notify_handle_resume[JOB_CTRL_NOTIFY_QUEUE_SIZE];
    JobCtrlNotifyHandle notify_handle_stopped[JOB_CTRL_NOTIFY_QUEUE_SIZE];
};

extern JobCtrl job_ctrl_svc;


#endif  // #ifndef SNAPMAKER_JOB_CTRL_SERVICE_H_
