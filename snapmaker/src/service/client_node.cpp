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
#include "system.h"
#include "motion.h"
#include "job_ctrl.h"
#include "client_node.h"


bool ClientNode::client_node_class_init;
SemaphoreHandle_t ClientNode::_lock;
ClientNode* ClientNode::client_node_tab[MAX_CLIENT_NODE_NUM];


void ClientNode::class_init(void) {
  LOG_I("client node: client node class int\r\n");
  for (uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    client_node_tab[i] = NULL;
  }

  _lock = xSemaphoreCreateMutex();
  if (!_lock) {
    LOG_E("client node: _lock create failed\r\n");
    while(1);
  }

  LOG_I("client node: register SACP cmd set and cmd id");
  host_hmi.apply_cmd_set_handle(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_NUM);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_START, NULL, sacp_cb);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_PAUSE, NULL, sacp_cb);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_RESUME, NULL, sacp_cb);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_STOP, NULL, sacp_cb);

  // register subscribe to SACP
  host_hmi.register_subscription(CMD_SET_JOB_CTRL, SUB_ID_JOB_CTRL_CUR_LINE_NUM, (void *)subscribe_cb, subscribe_cb);
  client_node_class_init = true;
}

err_code_t ClientNode::sacp_cb(void *obj, sacp_hmi_message_t *msg) {
  ClientNode *cn;
  
  cn = find_client_node(msg->peer);
  if (cn) {
    return cn->sacp_handle(msg);
  }
  else {
    ClientNode *new_cn = new ClientNode(msg->peer, msg->ch);
    if (!new_cn) {
      LOG_E("can not new this client node\r\n");
      sacp_send_result(msg, SACP_RET_NO_MEM);
      return E_FAILURE;
    }
    else {
      if (E_SUCCESS == add_client_node(new_cn)) {
        return new_cn->sacp_handle(msg);
      }
      else {
        LOG_E("can not add this client node to list\r\n");
        delete new_cn;
        sacp_send_result(msg, SACP_RET_NO_MEM);
        return E_FAILURE;
      }
    }
  }
}

err_code_t ClientNode::sacp_send_result(sacp_hmi_message_t *msg, uint8_t result) {
  sacp_hmi_message_t s_msg;
  uint8_t send_buf[SEND_BUF_SIZE];

  s_msg = *msg;
  s_msg.attr = SACP_MESSAGE_ATTR_ACK;
  s_msg.data = send_buf;
  send_buf[0] = result;
  s_msg.length = 1;
  return host_hmi.send(&s_msg);
}

ClientNode *ClientNode::find_client_node(uint32_t peer) {
  for(uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    if (peer == client_node_tab[i]->_peer) {
      return client_node_tab[i];
    }
  }
  return NULL;
}

err_code_t ClientNode::add_client_node(ClientNode *cn) {
  LOCK(_lock, 0);
  for(uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    if (NULL == client_node_tab[i]) {
      client_node_tab[i] = cn;
      UNLOCK(_lock);
      return E_SUCCESS;
    }
  }
  UNLOCK(_lock);
  return E_NO_RESRC;
}

bool ClientNode::get_batch_gcode(uint8_t client_id, req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode) {
  ClientNode *cn;
  
  cn = find_client_node(client_id);
  if (cn) {
    return cn->sacp_get_batch_gcode(req_batch_gcode, res_batch_gcode);
  }
  else {
    LOG_E("can not find this client");
    return false;
  }
}

uint16_t ClientNode::subscribe_cb(void *obj, uint8_t *buffer) {
  // TODO:
  return 0;
}

err_code_t ClientNode::init(void) {
  LOG_I("register softer time to handle hardtick\r\n");
}

void ClientNode::timer_cb(void *p) {
  LOG_I("Send hardtick if need\r\n");
}

bool ClientNode::sacp_get_batch_gcode(req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode) {
  sacp_hmi_message_t s_msg;
  uint16_t out_len;
  uint8_t send_buf[SEND_BUF_SIZE];
  // peer id? and send id?
  // seq not change
  s_msg.ch = _ch;
  s_msg.attr = SACP_MESSAGE_ATTR_ACK;
  s_msg.data = send_buf;
  _32_TO_LITTLE_STREAM(req_batch_gcode.line_num, send_buf);
  _16_TO_LITTLE_STREAM(req_batch_gcode.buf_len, send_buf + 4);
  s_msg.length = 6;
  if (E_SUCCESS != host_hmi.send_sync(&s_msg, send_buf, &out_len)) {
    if(out_len < 8){
      LOG_E("batch gcode response lenght error, must > 8, but get %d\r\n", out_len);
      return false;
    }
    res_batch_gcode.start_line_num = LITTLE_STREAM_TO_32(send_buf);
    res_batch_gcode.end_line_num = LITTLE_STREAM_TO_32(send_buf + 4);
    memcpy(res_batch_gcode.gcode_str, send_buf + 8, out_len - 8);
    return true;
  }
  else {
    LOG_E("send sync failed in when get gcode\r\n");
    return false;
  }
}

err_code_t ClientNode::sacp_handle(sacp_hmi_message_t *msg) {
  
  switch (msg->cmd_id) {
    case CMD_ID_JOB_CTRL_START:
      return req_start_job(msg);
    break;

    case CMD_ID_JOB_CTRL_PAUSE:
      return req_pause_job(msg);
    break;

    case CMD_ID_JOB_CTRL_RESUME:
      return req_resume_job(msg);
    break;

    case CMD_ID_JOB_CTRL_STOP:
      return req_stop_job(msg);
    break;

    default:
      LOG_E("Unkonw command id %d in command set %d\r\n", msg->cmd_id, msg->cmd_set);
      return E_FAILURE;
    break;
  }
}

err_code_t ClientNode::req_start_job(sacp_hmi_message_t *msg) {
  LOG_I("client %d request start a job\r\n", _peer);

  err_code_t ret;
  uint16_t str_len;
  toolHeadType type;
  struct GcodeFileInfo gfi;
  uint8_t *p = msg->data;

  // MD5
  str_len = LITTLE_STREAM_TO_16(p);
  if (str_len < GCODE_MD5_LENGTH) {
    LOG_E("MD5 length error\r\n");
    ret = SACP_RESULT_CODE_JOB_IVALID_GCODE_FILE;
    goto _out;
  }
  memcpy(gfi.MD5, p + 2, str_len);
  p += 2 + str_len;

  // gcode filename
  str_len = LITTLE_STREAM_TO_16(p);
  if (str_len > GCODE_FILE_NAME_SIZE-1) {
    LOG_E("file name too long\r\n");
    ret = SACP_RESULT_CODE_JOB_IVALID_GCODE_FILE;
    goto _out;
  }
  memcpy(gfi.name, p + 2, str_len);
  gfi.name[str_len] = '\0';
  p += 2 + str_len;

  // type 
  type = toolHeadType(p[0]);
  if (type >= TH_TYPE_UNKNOW) {
    LOG_E("unknow job type %d\r\n", type);
    ret = SACP_RET_UNSUPPORT_PARAM;
    goto _out;
  }

  // send starting
  ret = job_ctrl_svc.start(_peer, &gfi, type);

_out:
  return sacp_send_result(msg, ret);
}

err_code_t ClientNode::req_pause_job(sacp_hmi_message_t* msg) {
  LOG_I("client %d request pause a job\r\n");
  
  err_code_t ret;
  ret = sacp_send_result(msg, SACP_RET_EXECUTING);
  // TODO: do we need to check this ret?

  ret = job_ctrl_svc.pause();
  sacp_send_result(msg, ret);
  return ret;
}

err_code_t ClientNode::req_resume_job(sacp_hmi_message_t* msg) {
  LOG_I("client %d request resume a job\r\n");
  
  err_code_t ret;
  ret = sacp_send_result(msg, SACP_RET_EXECUTING);
  // TODO: do we need to check this ret?

  ret = job_ctrl_svc.resume(_peer);
  sacp_send_result(msg, ret);
  return ret;
}

err_code_t ClientNode::req_stop_job(sacp_hmi_message_t* msg) {
  LOG_I("client %d request stop a job\r\n");
  
  err_code_t ret;
  ret = sacp_send_result(msg, SACP_RET_EXECUTING);
  // TODO: do we need to check this ret?

  ret = job_ctrl_svc.stop();
  sacp_send_result(msg, ret);
  return ret;
}
