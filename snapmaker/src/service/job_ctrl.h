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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "../common/list.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
#include "../snapmaker.h"
#include "client_node.h"


#define E_JOB_FAILURE                     SACP_RET_FAILURE
#define E_JOB_LAST_GCODE_PACK             SACP_RET_JOB_LAST_GCODE_PACK
#define E_JOB_NOT_IN_IDLE_STATUS          SACP_RET_JOB_NOT_IN_IDLE_STATUS
#define E_JOB_NO_HOME                     SACP_RET_JOB_NO_HOME
#define E_JOB_IVALID_GCODE_FILE           SACP_RET_JOB_IVALID_GCODE_FILE
#define E_JOB_NOT_IN_WORKING_STATUS       SACP_RET_JOB_NOT_IN_WORKING_STATUS
#define E_JOB_NOT_IN_PAUSE_STATUS         SACP_RET_JOB_NOT_IN_PAUSE_STATUS
#define E_JOB_IVALID_POWER_LOSE_DATA      SACP_RET_JOB_IVALID_POWER_LOSE_DATA
#define E_JOB_POWER_LOSE_CHECK_FAILURE    SACP_RET_JOB_POWER_LOSE_CHECK_FAILURE
#define E_JOB_GCODE_FILE_NO_EXIT          SACP_RET_JOB_GCODE_FILE_NO_EXIT
#define E_JOB_SAVE_ENV_FAILURE            SACP_RET_JOB_SAVE_ENV_FAILURE
#define E_JOB_RESUME_ENV_FAILURE          SACP_RET_JOB_RESUME_ENV_FAILURE
#define E_JOB_UNKNOW_STOP_TPYE            SACP_RET_JOB_UNKNOW_STOP_TPYE
#define E_JOB_UNSUPPORT_PARAM             SACP_RET_UNSUPPORT_PARAM
#define E_JOB_UNMATCHED_TOOLHEAD          SACP_RET_JOB_UNMATCHED_TOOLHEAD

#define GCODE_MD5_LENGTH 32
#define GCODE_FILE_NAME_SIZE 128
#define JOB_LOCK_WAIT_TICK 100
#define TOOLHEAD_ENV_MAX_SIZE 128
#define GCODE_RB_SIZE 1024
#define RESUME_FEEDRATE 3000


// TODO: this type should define other
struct GcodeFileInfo {
  uint8_t MD5[GCODE_MD5_LENGTH];
  uint8_t name[GCODE_FILE_NAME_SIZE];
};

enum JobNotiy {
  PAUSE_FILM_RUNOUT,
  PAUSE_POWR_LOSE,
  PAUSE_EXCEPTION,
};

typedef float xyzijk_position_t[AXIS_NUM];

struct JobEnv {
  toolHeadType type;                                          /** job type :                                                            */
  bool gfi_valid;                                             /** gcode file information valid                                          */
  struct GcodeFileInfo gcode_file_info;                       /** gcode file information                                                */
  uint32_t req_line_num;                                      /** request line number                                                   */
  uint32_t cur_line_num;                                      /** current linenumber, use with gcode file to save the job point         */
  uint32_t time_elape;                                        /** time elaps from the job starting, in second                           */
  xyzijk_position_t current_pos;
  float print_feadrate;
  float travel_feadrate;
  bool g0g1_relative_mode;
  uint16_t bed_temp;
  uint32_t toolhead_env_buf_size; 
  uint8_t toolhead_env_buf[TOOLHEAD_ENV_MAX_SIZE];
};

class JobCtrl {
  // public methods
  public:
    JobCtrl(){};
    void init(void);
    void background_thread(void);                               /** main loop, to check all the event from system which will change current job status */

    // job control
    err_code_t start(uint8_t client_id, struct GcodeFileInfo *gcodeInfo, toolHeadType th_type);
    err_code_t pause(void);
    err_code_t resume(uint8_t client_id);
    err_code_t stop(void);

    // set & get
    err_code_t set_env(struct JobEnv &env);
    struct JobEnv get_env(void);
    toolHeadType get_type(void) { return _env.type; }
    struct GcodeFileInfo *get_gcode_info(void) { return _env.gfi_valid? &_env.gcode_file_info : NULL; }
    uint32_t get_cur_linenum(void) { return _env.cur_line_num; }
    uint32_t get_time_elaps(void) { return _env.time_elape; }
    void statistics_log_set(uint32_t interval_ms) { _statistics_log_interval_ms = interval_ms; };
    void statistics_output(void);

    // gcode
    bool consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line);
    bool gcode_file_info_check(struct GcodeFileInfo *gfi);

  // private methods
  private:
    err_code_t save_env(void);                                  /** save current job enviroment                                           */
    err_code_t resum_env(void);                                 /** resume saved enviroment to job                                        */
    err_code_t machine_standby(void);                           /** set the machine in standby status                                     */
    void notify();                                              /** notify the client about job status                                    */
    void quit_stop();                                           /** stop right now, when in emergency situation                           */
    void normal_stop();                                         /** stop when current block finish                                        */
    void issue_nodify(void);
    void get_gcodes_from_client(void);

    SemaphoreHandle_t _lock;                                    /** lock, TODO:should use the snapmaker's API, not the freeRTOS           */
    RingBuffer<uint8_t> _gcode_rb;                              /** ringbuffer for rx the gcode string                                    */
    uint8_t _client_id;                                         /** A pointer to client node,                                             */
    uint32_t _tick_ms;                                          /** use for periodically main loop                                        */
    uint32_t _resume_feedrate;                                  /** set the resume move feedrate                                          */
    struct JobEnv _env;                                         /** environment of this job, used to job resume                           */

    // use for state of self-inspection
    uint32_t _err_get_batch_gcode_cnt;
    uint32_t _statistics_log_interval_ms;
    uint32_t _statistics_log_last_tick_ms;
};

extern JobCtrl job_ctrl_svc;


#endif  // #ifndef SNAPMAKER_JOB_CTRL_SERVICE_H_
