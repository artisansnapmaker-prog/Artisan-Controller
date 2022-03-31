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
#include "motion_platform.h"
#include "client_node.h"


SemaphoreHandle_t ClientNode::_lock;
ClientNode* ClientNode::client_node_tab[MAX_CLIENT_NODE_NUM];

SemaphoreHandle_t ClientNode::sacp_msg_copy_lock;
sacp_hmi_message_t ClientNode::sacp_msg_copy[MAX_SACP_MSG_COPY];
bool ClientNode::sacp_msg_copy_occupy[MAX_SACP_MSG_COPY];


void ClientNode::class_init(void) {
  err_code_t ret;

  LOG_I("Client node: client node class int\r\n");
  for (uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    ClientNode *new_cn = new ClientNode(IVALID_PEER, IVALID_CH);
    configASSERT(new_cn);
    new_cn->init();
    client_node_tab[i] = new_cn;
  }

  _lock = xSemaphoreCreateMutex();
  configASSERT(_lock);

  sacp_msg_copy_lock = xSemaphoreCreateMutex();
  configASSERT(sacp_msg_copy_lock);
  for (uint32_t i = 0; i < MAX_SACP_MSG_COPY; i++) {
    sacp_msg_copy_occupy[i] = false;
  }

  LOG_I("Client node: register SACP cmd set and cmd id callback\r\n");
  ret = E_SUCCESS;
  // job control
  ret |= host_hmi.apply_cmd_set_handle(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_NUM);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_GET_GCODE_FILE_INFO, NULL, sacp_cb);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_START, NULL, sacp_cb);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_PAUSE, NULL, sacp_cb);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_RESUME, NULL, sacp_cb);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_STOP, NULL, sacp_cb);
  ret |= host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_SET_FEEDRATE_PERCENTAGE, NULL, sacp_cb);

  // register subscibtion
  LOG_I("Client node: register SACP subscription callback\r\n");
  ret |= host_hmi.register_subscription(  CMD_SET_JOB_CTRL, 
                                          SUB_ID_JOB_CTRL_CUR_LINE_NUM, 
                                          (void *)job_ctrl_linenum_sub_cb, 
                                          job_ctrl_linenum_sub_cb);

  if (E_SUCCESS != ret) {
    LOG_E("Client node: can not register sacp callback\r\n");
    while(1);
  }
}

err_code_t ClientNode::sacp_cb(void *obj, sacp_hmi_message_t *msg) {
  ClientNode *cn;

  cn = find_client_node(msg->peer);
  if (cn) {
    return cn->sacp_handle(msg);
  }
  else {
    ClientNode *new_cn = malloc_client_node(msg->peer, msg->ch);
    if (!new_cn) {
      LOG_E("Client node: can not malloc a client node\r\n");
      host_hmi.send_ack(msg, SACP_RET_NO_MEM);
      return E_FAILURE;
    }

    return new_cn->sacp_handle(msg);
  }
}

ClientNode *ClientNode::find_client_node(uint32_t peer) {
  for(uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    if (client_node_tab[i] && peer == client_node_tab[i]->_peer) {
      return client_node_tab[i];
    }
  }
  return NULL;
}

ClientNode * ClientNode::malloc_client_node(uint32_t peer, uint8_t ch) {
  ClientNode *ret = NULL;

  LOCK(_lock, 0);
  for(uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    if (client_node_tab[i] && IVALID_PEER == client_node_tab[i]->_peer) {
      client_node_tab[i]->_peer = peer;
      client_node_tab[i]->_ch = ch;
      ret = client_node_tab[i];
    }
  }
  UNLOCK(_lock);

  return ret;
}

err_code_t ClientNode::del_client_node(uint32_t peer) {
  ClientNode *cn = find_client_node(peer);
  if (!cn)
    return E_FAILURE;
  return del_client_node(cn);
}

err_code_t ClientNode::del_client_node(ClientNode *cn) {
  // TODO: should we need to lock?
  cn->_peer = IVALID_PEER;
  cn->_ch = IVALID_CH;
  return E_SUCCESS;
}

bool ClientNode::get_batch_gcode(uint8_t client_id, req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode) {
  ClientNode *cn;

  cn = find_client_node(client_id);
  if (cn) {
    return cn->sacp_get_batch_gcode(req_batch_gcode, res_batch_gcode);
  }
  else {
    LOG_E("Client node: can not find this client");
    return false;
  }
}

uint16_t ClientNode::job_ctrl_linenum_sub_cb(void *obj, uint8_t *buffer) {
  uint32_t ln;
  ln = job_ctrl_svc.get_cur_linenum();
  
  buffer[0] = E_SUCCESS;              // result
  _32_TO_LITTLE_STREAM(ln, buffer+1);   // line number
  return 5;
}

uint16_t ClientNode::sys_hardtick_sub_cb(void *obj, uint8_t *buffer) {
  buffer[0] = E_SUCCESS;
  buffer[1] = smprinter.get_sys_status();
  return 2;
}

err_code_t ClientNode::issue_client(uint8_t peer, uint8_t issue_ret) {
  // report status change reasone
  ClientNode *cn = find_client_node(peer);
  if (!cn) {
    return E_FAILURE;
  }

  sacp_hmi_message_t msg;
  uint8_t tx_buf[1];
  uint8_t rx_buf[8];
  uint16_t rx_len = 8;
  msg.cmd_set = CMD_SET_JOB_CTRL;
  msg.cmd_id = CMD_ID_JOB_CTRL_ISSUE;
  msg.ch = cn->_ch;
  msg.peer = cn->_peer;
  msg.data = tx_buf;
  msg.length = 1;
  msg.attr = 0;
  tx_buf[0] = issue_ret;

  if (E_SUCCESS != host_hmi.send_sync(&msg, rx_buf, &rx_len, 100, 3)) {
    LOG_E("Client node: Issue failure\r\n");
    return E_FAILURE;
  }
  if (E_SUCCESS != rx_buf[0]) {
    LOG_E("Client node: Issue failure\r\n");
    return E_FAILURE;
  }

  return E_SUCCESS;
}
ClientNode::ClientNode(uint32_t peer, uint8_t ch): _peer(peer), _ch(ch) {

}

err_code_t ClientNode::init(void) {
  return E_SUCCESS;
}

sacp_hmi_message_t *ClientNode::malloc_sacp_msg_node(void) {
  sacp_hmi_message_t *node = NULL;

  LOCK(sacp_msg_copy_lock, 0);
  for (uint32_t i = 0; i < MAX_SACP_MSG_COPY; i++) {
    if (!sacp_msg_copy_occupy[i]) {
      sacp_msg_copy_occupy[i] = true;
      node = &sacp_msg_copy[i];
      break;
    }
  }
  UNLOCK(sacp_msg_copy_lock);

  return node;
}

err_code_t ClientNode::free_sacp_msg_node(sacp_hmi_message_t *sacp_msg) {
  err_code_t ret = E_FAILURE;

  LOCK(sacp_msg_copy_lock, 0);
  for (uint32_t i = 0; i < MAX_SACP_MSG_COPY; i++) {
    if (sacp_msg == &sacp_msg_copy[i]) {
      sacp_msg_copy_occupy[i] = false;
      ret = E_SUCCESS;
      break;
    }
  }
  UNLOCK(sacp_msg_copy_lock);

  return ret;
}

void ClientNode::job_req_start_cb(void *p, uint8_t result) {
  sacp_hmi_message_t *copy_msg = (sacp_hmi_message_t *)p;
  configASSERT(copy_msg);

  if (SYSTEM_STATUS_STARTING == result) {
    LOG_I("TODO: client_node: send JOB STARTING ACK to client\r\n");
    // host_hmi.send_ack(copy_msg, SACP_RET_EXECUTING);
  }
  else if(SYSTEM_STATUS_PRINTING == result ||
          SYSTEM_STATUS_XY_CALIBRATING_PRINTING == result) {
    host_hmi.send_ack(copy_msg, SACP_RET_SUCCESS);
    free_sacp_msg_node(copy_msg);
  }
  else {
    host_hmi.send_ack(copy_msg, result);
    free_sacp_msg_node(copy_msg);
  }
}

void ClientNode::job_req_pause_cb(void *p, uint8_t result) {
  sacp_hmi_message_t *copy_msg = (sacp_hmi_message_t *)p;
  configASSERT(copy_msg);

  if (SYSTEM_STATUS_PAUSING == result) {
    LOG_I("TODO: client_node: send JOB PAUSING ACK to client\r\n");
    // host_hmi.send_ack(copy_msg, SACP_RET_EXECUTING);
  }
  else if(SYSTEM_STATUS_PAUSED == result) {
    LOG_I("client_node: send JOB PAUSED OK ACK to client\r\n");
    host_hmi.send_ack(copy_msg, SACP_RET_SUCCESS);
    free_sacp_msg_node(copy_msg);
  }
  else {
    LOG_I("client_node: pause failure\r\n");
    host_hmi.send_ack(copy_msg, result);
    free_sacp_msg_node(copy_msg);
  }
}

void ClientNode::job_req_resume_cb(void *p, uint8_t result) {
  sacp_hmi_message_t *copy_msg = (sacp_hmi_message_t *)p;
  configASSERT(copy_msg);

  if (SYSTEM_STATUS_RESUMING == result) {
    LOG_I("TODO: client_node: send JOB RESUMING ACK to client\r\n");
    // host_hmi.send_ack(copy_msg, SACP_RET_EXECUTING);
  }
  else if(SYSTEM_STATUS_PRINTING == result) {
    host_hmi.send_ack(copy_msg, SACP_RET_SUCCESS);
    free_sacp_msg_node(copy_msg);
  }
  else {
    host_hmi.send_ack(copy_msg, result);
    free_sacp_msg_node(copy_msg);
  }
}

void ClientNode::job_req_stop_cb(void *p, uint8_t result) {
  sacp_hmi_message_t *copy_msg = (sacp_hmi_message_t *)p;
  configASSERT(copy_msg);

  if (SYSTEM_STATUS_STOPING == result) {
    LOG_I("TODO: client_node: send JOB STOPING ACK to client\r\n");
    // host_hmi.send_ack(copy_msg, SACP_RET_EXECUTING);
  }
  else if(SYSTEM_STATUS_IDLE == result || 
          SYSTEM_STATUS_XY_CALIBRATING == result) {
    host_hmi.send_ack(copy_msg, SACP_RET_SUCCESS);
    free_sacp_msg_node(copy_msg);
  }
  else {
    host_hmi.send_ack(copy_msg, result);
    free_sacp_msg_node(copy_msg);
  }
}

void ClientNode::timer_cb(void *p) {
  LOG_I("Send hardtick if need\r\n");
}

bool ClientNode::sacp_get_batch_gcode(req_batch_gcode_t &req_batch_gcode, res_batch_gcode_t &res_batch_gcode) {
  err_code_t ret;
  uint16_t out_len;
  sacp_hmi_message_t s_msg;
  uint8_t buf[SEND_BUF_SIZE];

  // 11 is the start of linenumber end of linenumber and so on
  if (req_batch_gcode.buf_len + 11 > SEND_BUF_SIZE) {
    LOG_E("client_node: sacp_get_batch_gcode req buf len too large\r\n");
    return false;
  }

  s_msg.ch = _ch;
  s_msg.peer = SACP_HOST_ID_SCREEN;
  s_msg.attr = 0;
  s_msg.data = buf;
  s_msg.cmd_set = CMD_SET_JOB_CTRL;
  s_msg.cmd_id = CMD_ID_JOB_CTRL_REQ_GCODE;
  _32_TO_LITTLE_STREAM(req_batch_gcode.line_num, buf);
  _16_TO_LITTLE_STREAM(req_batch_gcode.buf_len, buf + 4);
  s_msg.length = 6;
  out_len = SEND_BUF_SIZE;
  ret = host_hmi.send_sync(&s_msg, buf, &out_len, 2000, 2);
  if (E_SUCCESS == ret) {
    if(out_len < 8){
      LOG_E("Client node: batch gcode response lenght error, must > 8, but get %d\r\n", out_len);
      return false;
    }
    uint8_t *p = buf;
    res_batch_gcode.result = p[0]; p += 1;
    res_batch_gcode.start_line_num = LITTLE_STREAM_TO_32(p); p += 4;
    res_batch_gcode.end_line_num = LITTLE_STREAM_TO_32(p); p+= 4;
    uint16_t str_len = LITTLE_STREAM_TO_16(p); p += 2;
    // LOG_I("client_node: start line %d, end line %d, strlen %d\r\n", res_batch_gcode.start_line_num, res_batch_gcode.end_line_num, str_len);
    // LOG_I("gcode string: %s\r\n", (char *)p);
    memcpy(res_batch_gcode.gcode_str, p, str_len); p += str_len;
    res_batch_gcode.gcode_str[str_len] = '\0';
    return true;
  }
  else {
    LOG_E("Client node: send sync failed in when get gcode, return %d\r\n", ret);
    return false;
  }
}

err_code_t ClientNode::sacp_handle(sacp_hmi_message_t *msg) {

  switch (msg->cmd_id) {
    case CMD_ID_JOB_GET_GCODE_FILE_INFO:
      return get_gcode_info(msg);
    break;

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

    case CMD_ID_JOB_SET_FEEDRATE_PERCENTAGE:
      return req_set_feedrate_percentage(msg);
    break;

    default:
      LOG_E("Client node: Unkonw command id %d in command set %d\r\n", msg->cmd_id, msg->cmd_set);
      return E_FAILURE;
    break;
  }
}

err_code_t ClientNode::get_gcode_info(sacp_hmi_message_t* msg) {
  uint16_t str_len;
  struct GcodeFileInfo *gfi;
  uint8_t *p = msg->data;

  gfi = job_ctrl_svc.get_gcode_info();
  if (gfi) {
    p[0] = SACP_RET_SUCCESS;

    _16_TO_LITTLE_STREAM(GCODE_MD5_LENGTH, p);
    p += 2;
    memcpy(p, gfi->MD5, GCODE_MD5_LENGTH);
    p += GCODE_MD5_LENGTH;

    str_len = strlen((char *)gfi->name);
    _16_TO_LITTLE_STREAM(str_len, p);
    p += 2;
    memcpy(p, gfi->name, str_len);
    p += str_len;
  }
  else {
    p[0] = SACP_RET_JOB_GCODE_FILE_NO_EXIT;

    // dummy data
    _16_TO_LITTLE_STREAM(GCODE_MD5_LENGTH, p);
    p += 2;
    memset(p, 0, GCODE_MD5_LENGTH);
    p += GCODE_MD5_LENGTH;

    _16_TO_LITTLE_STREAM(GCODE_FILE_NAME_SIZE, p);
    p += 2;
    memset(p, 0, GCODE_FILE_NAME_SIZE);
    p += GCODE_FILE_NAME_SIZE;
  }
  msg->length = p - msg->data;
  return host_hmi.send(msg);
}

err_code_t ClientNode::req_start_job(sacp_hmi_message_t *msg) {
  err_code_t ret;
  uint16_t str_len;
  toolHeadType type;
  struct GcodeFileInfo gfi;
  sacp_hmi_message_t *msg_cp;
  uint8_t *p;

  LOG_I("client_node: client %d request start a job\r\n", _peer);

  p = msg->data;
  // check MD5
  str_len = LITTLE_STREAM_TO_16(p);
  p += 2;
  if (str_len < GCODE_MD5_LENGTH) {
    LOG_E("Client node: MD5 length error\r\n");
    return host_hmi.send_ack(msg, SACP_RET_JOB_IVALID_GCODE_FILE);
  }
  memcpy(gfi.MD5, p, str_len);
  p += str_len;

  // check gcode filename
  str_len = LITTLE_STREAM_TO_16(p);
  p += 2;
  if (str_len > GCODE_FILE_NAME_SIZE-1) {
    LOG_E("Client node: file name too long\r\n");
    return host_hmi.send_ack(msg, SACP_RET_JOB_IVALID_GCODE_FILE);
  }
  memcpy(gfi.name, p, str_len);
  gfi.name[str_len] = '\0';
  p += str_len;

  // check print type: fdm cnc laser
  type = toolHeadType(p[0]);
  if (type >= TH_TYPE_UNKNOW) {
    LOG_E("Client node: unknow job type %d\r\n", type);
    return host_hmi.send_ack(msg, SACP_RET_UNSUPPORT_PARAM);
  }

  // starting
  msg_cp = malloc_sacp_msg_node();
  if (!msg_cp) {
    LOG_E("client_node: can not malloc sacp msg copy\r\n");
    return host_hmi.send_ack(msg, SACP_RET_NO_RESC);
  }
  *msg_cp = *msg;
  ret = job_ctrl_svc.req_start(_peer, &gfi, type, job_req_start_cb, msg_cp);
  if (E_SUCCESS != ret) {
    free_sacp_msg_node(msg_cp);
    return host_hmi.send_ack(msg, ret);
  }

  return E_SUCCESS;
}

err_code_t ClientNode::req_pause_job(sacp_hmi_message_t* msg) {
  err_code_t ret;
  sacp_hmi_message_t *msg_cp;

  LOG_I("Client node: %d client %d request pause a job <<<<<<<<<<<>>>>>>>>>>>\r\n", millis());
  msg_cp = malloc_sacp_msg_node();
  if (!msg_cp) {
    LOG_E("client_node: can not malloc sacp msg copy\r\n");
    return host_hmi.send_ack(msg, SACP_RET_NO_RESC);
  }
  *msg_cp = *msg;
  ret = job_ctrl_svc.req_pause(PAUSE_CLIENT_REQ, job_req_pause_cb, msg_cp);
  if (E_SUCCESS != ret) {
    free_sacp_msg_node(msg_cp);
    return host_hmi.send_ack(msg, ret);
  }

  return E_SUCCESS;
}

err_code_t ClientNode::req_resume_job(sacp_hmi_message_t* msg) {
  err_code_t ret;
  sacp_hmi_message_t *msg_cp;

  LOG_I("Client node: client %d request resume a job\r\n");
  msg_cp = malloc_sacp_msg_node();
  if (!msg_cp) {
    LOG_E("client_node: can not malloc sacp msg copy\r\n");
    return host_hmi.send_ack(msg, SACP_RET_NO_RESC);
  }
  *msg_cp = *msg;
  ret = job_ctrl_svc.req_resume(_peer, job_req_resume_cb, msg_cp);
  if (E_SUCCESS != ret) {
    free_sacp_msg_node(msg_cp);
    return host_hmi.send_ack(msg, ret);
  }

  return E_SUCCESS;
}

err_code_t ClientNode::req_stop_job(sacp_hmi_message_t* msg) {
  err_code_t ret;
  sacp_hmi_message_t *msg_cp;

  LOG_I("Client node: client %d request stop a job\r\n");
  msg_cp = malloc_sacp_msg_node();
  if (!msg_cp) {
    LOG_E("client_node: can not malloc sacp msg copy\r\n");
    return host_hmi.send_ack(msg, SACP_RET_NO_RESC);
  }
  *msg_cp = *msg;
  // ret = job_ctrl_svc.req_stop(STOP_CLIENT_REQ, SACP_JOB_PAUSE_ISSUE_RET_STOP_CLIENT_REQ, job_req_stop_cb, msg_cp);
  ret = job_ctrl_svc.req_stop(STOP_CLIENT_REQ, SACP_JOB_PAUSE_ISSUE_RET_FINISH, job_req_stop_cb, msg_cp);
  if (E_SUCCESS != ret) {
    free_sacp_msg_node(msg_cp);
    return host_hmi.send_ack(msg, ret);
  }

  return E_SUCCESS;
}

err_code_t ClientNode::req_set_feedrate_percentage(sacp_hmi_message_t* msg) {
  return E_SUCCESS;
}