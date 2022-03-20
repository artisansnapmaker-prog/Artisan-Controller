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


#define MAX_CLIENT_NODE_NUM                           4
#define MAX_SACP_MSG_COPY                             2
#define SEND_BUF_SIZE                                 256
// TODO: this should define in the SACP
#define IVALID_PEER                                   0xFFFFFFFF
#define IVALID_CH                                     0xFF

#define CMD_SET_JOB_CTRL                              (0xAC)
#define CMD_ID_JOB_GET_GCODE_FILE_INFO                (0x00)
#define CMD_ID_JOB_CTRL_ISSUE                         (0x01)
#define CMD_ID_JOB_CTRL_REQ_GCODE                     (0x02)
#define CMD_ID_JOB_CTRL_START                         (0x03)
#define CMD_ID_JOB_CTRL_PAUSE                         (0x04)
#define CMD_ID_JOB_CTRL_RESUME                        (0x05)
#define CMD_ID_JOB_CTRL_STOP                          (0x06)
#define CMD_ID_JOB_SET_FEEDRATE_PERCENTAGE            (0x0e)
#define CMD_ID_JOB_CTRL_NUM                           (11)
#define SUB_ID_JOB_CTRL_CUR_LINE_NUM                  (0xA0)

#define CMD_SET_SYS                                   (1)
#define CMD_ID_SYS_SET_ECHO_LOG                       (10)
#define CMD_ID_SYS_SET_PC_CH_PRO                      (11)  // TODO: should we need this cmd id?
#define CMD_ID_SYS_SET_DEBUG_MODE                     (12)  // TODO: should we need this cmd id?
#define CMD_ID_SYS_NUM                                (3)
#define SUB_ID_SYS_HARDTICK                           (0xA0)

#define SACP_RET_SUCCESS                              E_SUCCESS
#define SACP_RET_EXECUTING                            E_EXECUTING
#define SACP_RET_TRANS_TIMEOUT                        E_TRANS_TIMEOUT
#define SACP_RET_EXECUTING_TIMEOUT                    E_EXE_TIMEOUT
#define SACP_RET_UNSUPPORT_CMD_SET                    E_INVALID_CMD_SET
#define SACP_RET_UNSUPPORT_CMD_ID                     E_INVALID_CMD_ID
#define SACP_RET_UNSUPPORT_PARAM                      E_PARAM
#define SACP_RET_UNSUPPORT_MOUDLE_KEY                 E_INVALID_MODULE_KEY
#define SACP_RET_NO_MEM                               E_NO_MEM
#define SACP_RET_NO_RESC                              E_NO_RESRC
#define SACP_RET_FAILURE                              E_FAILURE
#define SACP_RET_BUSY                                 E_BUSY

#define SACP_RET_PRIVATE_BASE                         (PRIVATE_ERROR_BASE)
#define SACP_RET_JOB_LAST_GCODE_PACK                  (SACP_RET_PRIVATE_BASE + 1)
#define SACP_RET_JOB_NOT_IN_IDLE_STATUS               (SACP_RET_PRIVATE_BASE + 2)
#define SACP_RET_JOB_NO_HOME                          (SACP_RET_PRIVATE_BASE + 3)
#define SACP_RET_JOB_IVALID_GCODE_FILE                (SACP_RET_PRIVATE_BASE + 4)
#define SACP_RET_JOB_NOT_IN_WORKING_STATUS            (SACP_RET_PRIVATE_BASE + 5)
#define SACP_RET_JOB_NOT_IN_PAUSE_STATUS              (SACP_RET_PRIVATE_BASE + 6)
#define SACP_RET_JOB_IVALID_POWER_LOSE_DATA           (SACP_RET_PRIVATE_BASE + 7)
#define SACP_RET_JOB_POWER_LOSE_CHECK_FAILURE         (SACP_RET_PRIVATE_BASE + 8)
#define SACP_RET_JOB_GCODE_FILE_NO_EXIT               (SACP_RET_PRIVATE_BASE + 10)
#define SACP_RET_JOB_SAVE_ENV_FAILURE                 (SACP_RET_PRIVATE_BASE + 11)
#define SACP_RET_JOB_RESUME_ENV_FAILURE               (SACP_RET_PRIVATE_BASE + 13)
#define SACP_RET_JOB_UNKNOW_STOP_TPYE                 (SACP_RET_PRIVATE_BASE + 14)
#define SACP_RET_JOB_BUSY                             (SACP_RET_PRIVATE_BASE + 15)
#define SACP_RET_JOB_UNMATCHED_TOOLHEAD               (SACP_RET_PRIVATE_BASE + 16)

#define SACP_JOB_PAUSE_ISSUE_RET_FINISH                         (0)
#define SACP_JOB_PAUSE_ISSUE_RET_GCODE_PAUSE                    (1)
#define SACP_JOB_PAUSE_ISSUE_RET_GCODE_FILAMENT_RUNOUT          (2)
#define SACP_JOB_PAUSE_ISSUE_RET_FILAMENT_RUNOUT                (3)
#define SACP_JOB_PAUSE_ISSUE_RET_STALL_PROTECTION               (4)
#define SACP_JOB_PAUSE_ISSUE_RET_ABNORMAL_TEMP_PROTECTION       (5)
#define SACP_JOB_PAUSE_ISSUE_RET_IVALID_GCODE_LINE_NUMBER       (6)
#define SACP_JOB_PAUSE_ISSUE_RET_GET_GCODE_FAILURE              (7)

//Types of event function callbacks
typedef std::function<err_code_t(sacp_hmi_message_t&)> evevnt_cb_f;

typedef struct {
  uint32_t line_num;
  uint16_t buf_len;
} req_batch_gcode_t;

typedef struct {
  err_code_t result;
  uint32_t start_line_num;
  uint32_t end_line_num;
  uint8_t *gcode_str;
} res_batch_gcode_t;

class ClientNode {
  // Class define
  public:
    static void class_init(void);

    static ClientNode *find_client_node(uint32_t peer);
    static ClientNode *malloc_client_node(uint32_t peer, uint8_t ch);
    static err_code_t del_client_node(uint32_t peer);
    static err_code_t del_client_node(ClientNode *cn);

    static err_code_t sacp_cb(void *obj, sacp_hmi_message_t *);
    static bool get_batch_gcode(uint8_t client_id, req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode);
    static uint16_t job_ctrl_linenum_sub_cb(void *obj, uint8_t *buffer);
    static uint16_t sys_hardtick_sub_cb(void *obj, uint8_t *buffer);
    static err_code_t issue_client(uint8_t peer, uint8_t issue_ret);

  private:
    static SemaphoreHandle_t _lock;
    static ClientNode* client_node_tab[MAX_CLIENT_NODE_NUM];

  // Instance define
  public:
    ClientNode(uint32_t peer, uint8_t ch);
    err_code_t init(void);
    void timer_cb(void *p);
    bool sacp_get_batch_gcode(req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode);

    uint32_t _peer;
    uint8_t _ch;

  private:
    typedef std::function<void(int)> job_req_notify_cb_t;
    job_req_notify_cb_t req_start_cb;
    job_req_notify_cb_t req_pause_cb;
    job_req_notify_cb_t req_resume_cb;
    job_req_notify_cb_t req_stop_cb;

    SemaphoreHandle_t sacp_msg_copy_lock;
    sacp_hmi_message_t sacp_msg_copy[MAX_SACP_MSG_COPY];
    bool sacp_msg_copy_occupy[MAX_SACP_MSG_COPY];

    sacp_hmi_message_t *malloc_sacp_msg_node(void);
    err_code_t free_sacp_msg_node(sacp_hmi_message_t*);

    err_code_t sacp_handle(sacp_hmi_message_t*);
    err_code_t get_gcode_info(sacp_hmi_message_t*);
    err_code_t req_start_job(sacp_hmi_message_t*);
    err_code_t req_pause_job(sacp_hmi_message_t*);
    err_code_t req_resume_job(sacp_hmi_message_t*);
    err_code_t req_stop_job(sacp_hmi_message_t*);
    err_code_t req_set_feedrate_percentage(sacp_hmi_message_t* msg);
    void job_req_start_cb(sacp_hmi_message_t *copy_msg, uint8_t result);
    void job_req_pause_cb(sacp_hmi_message_t *copy_msg, uint8_t result);
    void job_req_resume_cb(sacp_hmi_message_t *copy_msg, uint8_t result);
    void job_req_stop_cb(sacp_hmi_message_t *copy_msg, uint8_t result);
};







#endif  // #ifndef SNAPMAKER_CLIENT_NODE_H_
