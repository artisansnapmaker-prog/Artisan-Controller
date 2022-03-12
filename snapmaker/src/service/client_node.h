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

#ifndef SNAPMAKER_CLIENT_NODE_H_
#define SNAPMAKER_CLIENT_NODE_H_


#include <functional>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
#include "../host/sacp_hmi.h"
#include "job_ctrl.h"


#define MAX_CLIENT_NODE_NUM 4
#define SEND_BUF_SIZE 256

#define CMD_SET_JOB_CTRL                  (0xAC)
#define CMD_ID_JOB_CTRL_ISSUE             (0x01)
#define CMD_ID_JOB_CTRL_REQ_GCODE         (0x02)
#define CMD_ID_JOB_CTRL_START             (0x03)
#define CMD_ID_JOB_CTRL_PAUSE             (0x04)
#define CMD_ID_JOB_CTRL_RESUME            (0x05)
#define CMD_ID_JOB_CTRL_STOP              (0x06)
#define SUB_ID_JOB_CTRL_CUR_LINE_NUM      (0xA0)
#define SUB_NUM_JOB_CTRL                  (1)

// TODO: this should define in the SACP
#define SACP_RESULT_CODE_APP_BASE                       (200)

#define SACP_RESULT_CODE_JOB_LAST_GCODE_PACK            (SACP_RESULT_CODE_APP_BASE + 1)
#define SACP_RESULT_CODE_JOB_NOT_IN_IDLE_STATUS         (SACP_RESULT_CODE_APP_BASE + 2)
#define SACP_RESULT_CODE_JOB_NO_HOME                    (SACP_RESULT_CODE_APP_BASE + 3)
#define SACP_RESULT_CODE_JOB_IVALID_GCODE_FILE          (SACP_RESULT_CODE_APP_BASE + 4)
#define SACP_RESULT_CODE_JOB_NOT_IN_WORKING_STATUS      (SACP_RESULT_CODE_APP_BASE + 5)
#define SACP_RESULT_CODE_JOB_NOT_IN_PAUSE_STATUS        (SACP_RESULT_CODE_APP_BASE + 6)
#define SACP_RESULT_CODE_JOB_IVALID_POWER_LOSE_DATA     (SACP_RESULT_CODE_APP_BASE + 7)
#define SACP_RESULT_CODE_JOB_POWER_LOSE_CHECK_FAILURE   (SACP_RESULT_CODE_APP_BASE + 8)
#define SACP_RESULT_CODE_JOB_GCODE_FILE_NO_EXIT         (SACP_RESULT_CODE_APP_BASE + 10)
#define SACP_RESULT_CODE_JOB_SAVE_ENV_FAILURE           (SACP_RESULT_CODE_APP_BASE + 11)
#define SACP_RESULT_CODE_JOB_RESUME_ENV_FAILURE         (SACP_RESULT_CODE_APP_BASE + 13)
#define SACP_RESULT_CODE_JOB_UNKNOW_STOP_TPYE           (SACP_RESULT_CODE_APP_BASE + 14)
#define SACP_RESULT_CODE_JOB_BUSY                       (SACP_RESULT_CODE_APP_BASE + 15)

// TODO: this result code for sacp should define in the sacp module
enum SacpResultCode {
  SACP_RET_SUCCESS = 0,
  SACP_RET_EXECUTING = 1,
  SACP_RET_TRANS_TIMEOUT = 2,
  SACP_RET_EXECUTING_TIMEOUT = 3,
  SACP_RET_UNSUPPORT_CMD_SET = 4,
  SACP_RET_UNSUPPORT_CMD_ID = 5,
  SACP_RET_UNSUPPORT_PARAM = 6,
  SACP_RET_UNSUPPORT_MOUDLE_KEY = 7,
  SACP_RET_NO_MEM = 8,
  SACP_RET_NO_RESC = 9,
};

//Types of event function callbacks
typedef std::function<err_code_t(sacp_hmi_message_t&)> evevnt_cb_f;
typedef struct {
  uint8_t cmd_id;
  evevnt_cb_f cb;
} event_cb_itme_t;

typedef struct {
  uint32_t line_num;
  uint16_t buf_len;
} req_batch_gcode_t;

typedef struct {
  uint32_t start_line_num;
  uint32_t end_line_num;
  uint8_t *gcode_str;
} res_batch_gcode_t;

class ClientNode {
  // Class define
  public:
    static void class_init(void);

    static ClientNode *find_client_node(uint32_t peer);
    static err_code_t add_client_node(ClientNode *cn);
    // TODO: client node delete() and client node?

    static err_code_t sacp_cb(void *obj, sacp_hmi_message_t *);
    static err_code_t sacp_send_result(sacp_hmi_message_t *msg, uint8_t result);
    static bool get_batch_gcode(uint8_t client_id, req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode);
    static uint16_t subscribe_cb(void *obj, uint8_t *buffer);

  private:
    static bool client_node_class_init;
    static SemaphoreHandle_t _lock;
    static ClientNode* client_node_tab[MAX_CLIENT_NODE_NUM];

  // Instance define
  public:
    ClientNode(uint32_t peer, uint8_t ch): _peer(peer), _ch(ch) {}
    err_code_t init(void);
    void timer_cb(void *p);
    bool sacp_get_batch_gcode(req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode);

  private:
    err_code_t sacp_handle(sacp_hmi_message_t*);
    err_code_t req_start_job(sacp_hmi_message_t*);
    err_code_t req_pause_job(sacp_hmi_message_t*);
    err_code_t req_resume_job(sacp_hmi_message_t*);
    err_code_t req_stop_job(sacp_hmi_message_t*);

  private:
    uint32_t _peer;
    uint8_t _ch;
};







#endif  // #ifndef SNAPMAKER_CLIENT_NODE_H_
