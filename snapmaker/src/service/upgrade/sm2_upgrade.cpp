/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2022 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Controller2022-Marlin)
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
#include "../../snapmaker.h"
#include "upgrade_controller_to_module.h"
#include "sm2_upgrade.h"


#define UPGRADE_SM2_MODULE_START_REQ              (0x06)
#define UPGRADE_SM2_MODULE_START_ACK              (0x07)
#define UPGRADE_SM2_MODULE_READY_REQ              (0x15)
#define UPGRADE_SM2_MODULE_READY_ACK              (0x16)
#define UPGRADE_SM2_MODULE_START_TRANS            (0x17)
#define UPGRADE_SM2_MODULE_TRANS_REQ              (0x09)
#define UPGRADE_SM2_MODULE_TRANS_ACK              (0x08)
#define UPGRADE_SM2_MODULE_END                    (0x0A)

#define UPGRADE_SM2_MODULE_NEED_UPGRADE           (0x01)
#define UPGRADE_SM2_MODULE_READY                  (0x01)


static UpgradeModuleHandle *sm2_func_tab = NULL;
static module_info_t *mit = NULL;
static uint16_t send_pack_index;
static uint32_t fw_file_offset;


static err_code_t sm2_module_sacp_callback(void *obj, sacp_module_message_t *msg) {
  // FUN_LOG();
  switch (msg->cmd_id) {
    case MODULE_EXT_CMD_START_UPGRADE_ACK:
      return sm2_module_upgrade_start_ack_cb(obj, msg);
    break;

    case MODULE_EXT_CMD_TRANS_FW_ACK:
      return sm2_module_upgrade_trans_req(obj, msg);
    break;

    case MODULE_EXT_CMD_GET_UPGRADE_STATUS_ACK:
      return sm2_module_upgrade_ready_ack(obj, msg);
    break;
  }

  return E_SUCCESS;
}

err_code_t sm2_module_upgrade_init(void) {
  err_code_t ret;

  ret = E_SUCCESS;
  ret |= host_can_cfg.register_callback( MODULE_EXT_CMD_START_UPGRADE_ACK, 
                                        (void *)NULL, 
                                        sm2_module_sacp_callback);
  ret |= host_can_cfg.register_callback( MODULE_EXT_CMD_TRANS_FW_ACK, 
                                        (void *)NULL, 
                                        sm2_module_sacp_callback);
  ret |= host_can_cfg.register_callback(MODULE_EXT_CMD_GET_UPGRADE_STATUS_ACK, 
                                        (void *)NULL, 
                                        sm2_module_sacp_callback);
  
  return ret;                          
}

err_code_t sm2_module_upgrade_handle_init(UpgradeModuleHandle *func_tab) {
  // FUN_LOG();
  if (!func_tab) {
    LOG_E("[%s] func_tab is null\n", __FUNCTION__);
    return E_PARAM;
  }
  send_pack_index = 0;
  mit = NULL;
  fw_file_offset = 0;

  taskENTER_CRITICAL();
  func_tab->start_req = sm2_module_upgrade_start_req;
  func_tab->ready_req = sm2_module_upgrade_ready_req;
  func_tab->trans_ack = sm2_module_upgrade_trans_ack;
  func_tab->end_req   = sm2_module_upgrade_end_req;
  sm2_func_tab = func_tab;
  taskEXIT_CRITICAL();

  return E_SUCCESS;
}

void sm2_module_upgrade_handle_deinit(void) {
  // FUN_LOG();
  taskENTER_CRITICAL();
  sm2_func_tab = NULL;
  taskEXIT_CRITICAL();
}

err_code_t sm2_module_upgrade_start_req(pack_info_t *p, module_info_t *m) {
  // FUN_LOG();
  uint8_t buffer[BOOT_PACK_FW_VER_STR_LEN + 8];
  sacp_module_message_t msg;
  if (!m)
    return E_FAILURE;

  mit = m;
  msg.peer   = mit->mac;
  msg.ch     = mit->ch;
  msg.cmd_id = MODULE_EXT_CMD_START_UPGRADE_REQ;
  msg.data   = buffer;
  msg.data[0] = p->upgrade_ctrl_flag;
  uint32_t len = 0;
  while(len < BOOT_PACK_FW_VER_STR_LEN) {
    msg.data[2 + len] = p->fw_ver_str[len];
    len++;
    if('\0' == p->fw_ver_str[len])
      break;
  }
  msg.length = len + 1;

  fw_file_offset = 0;

  return host_can_cfg.send(&msg);
}

err_code_t sm2_module_upgrade_start_ack_cb(void *obj, sacp_module_message_t *msg) {
  // FUN_LOG();
  err_code_t ret = E_FAILURE;

  if (!msg || msg->length < 1){
    return ret;
  }

  if (sm2_func_tab && sm2_func_tab->start_ack) {
    uint8_t ack = msg->data[0] == UPGRADE_SM2_MODULE_NEED_UPGRADE ? E_SUCCESS : E_FAILURE;
    sm2_func_tab->start_ack(ack);
    ret = E_SUCCESS;
  }

  return ret;
}

err_code_t sm2_module_upgrade_ready_req(void) {
  // FUN_LOG();
  sacp_module_message_t msg;

  msg.peer   = mit->mac;
  msg.ch     = mit->ch;
  msg.cmd_id = MODULE_EXT_CMD_GET_UPGRADE_STATUS_REQ;
  msg.data   = NULL;
  msg.length = 0;

  return host_can_cfg.send(&msg);
}

err_code_t sm2_module_upgrade_ready_ack(void *obj, sacp_module_message_t *msg) {
  // FUN_LOG();
  err_code_t ret = E_FAILURE;

  if (!msg || msg->length < 1){
    return ret;
  }

  if (sm2_func_tab && sm2_func_tab->ready_ack) {
    uint8_t ack = msg->data[0] == UPGRADE_SM2_MODULE_READY ? E_SUCCESS : E_FAILURE;
    sm2_func_tab->ready_ack(ack);
    ret = E_SUCCESS;
  }
  return ret;  
}

err_code_t sm2_module_upgrade_start_trans(void) {
  // FUN_LOG();
  sacp_module_message_t msg;

  msg.peer   = mit->mac;
  msg.ch     = mit->ch;
  msg.cmd_id = MODULE_EXT_CMD_INFORM_UPGRADE_START;
  msg.data   = NULL;
  msg.length = 0;

  return host_can_cfg.send(&msg);  
}

err_code_t sm2_module_upgrade_trans_req(void *obj, sacp_module_message_t *msg) {
  // FUN_LOG();
  err_code_t ret;
  uint16_t get_pack_index;

  ret = E_FAILURE;
  if (!msg || msg->length < 3) {
    LOG_E("[%s] got a invalid parameter\n", __FUNCTION__);
    return E_PARAM;
  }

  get_pack_index = msg->data[1] << 8 | msg->data[2];
  LOG_I("[%s] get_pack_index: %d\n", __FUNCTION__, get_pack_index);

  // if (get_pack_index != send_pack_index) {
  //   LOG_E("[%s] mismatched package index, get_pack_index:%d send_pack_index:%d\n",
  //         __FUNCTION__, get_pack_index, send_pack_index);
  // }

  taskENTER_CRITICAL();
  send_pack_index = get_pack_index;
  taskEXIT_CRITICAL();

  if (sm2_func_tab && sm2_func_tab->trans_req) {
    sm2_func_tab->trans_req(fw_file_offset, UPGRADE_CM_TRANS_BUF_SIZE);
    ret = E_SUCCESS;
  }
  else {
    LOG_E("[%s] there is no corresponding function interface\n", __FUNCTION__);
  }

  return ret;
}

err_code_t sm2_module_upgrade_trans_ack(uint32_t offset, uint8_t *data, uint32_t len) {
  // FUN_LOG();
  uint8_t buffer[UPGRADE_CM_TRANS_BUF_SIZE + 1];
  sacp_module_message_t msg;

  if (!data || len > UPGRADE_CM_TRANS_BUF_SIZE) {
    return E_FAILURE;
  }

  msg.peer    = mit->mac;
  msg.ch      = mit->ch;
  msg.cmd_id  = MODULE_EXT_CMD_TRANS_FW_REQ;
  msg.data    = buffer;
  msg.data[0] = 0x00;
  memcpy(msg.data + 1, data, len);
  msg.length  = len + 1;

  taskENTER_CRITICAL();
  fw_file_offset += len;
  taskEXIT_CRITICAL();

  return host_can_cfg.send(&msg);
}

err_code_t sm2_module_upgrade_end_req(uint8_t ret) {
  // FUN_LOG();
  uint8_t buffer[8];
  sacp_module_message_t msg;

  msg.peer    = mit->mac;
  msg.ch      = mit->ch;
  msg.cmd_id  = MODULE_EXT_CMD_END_UPGRADE_REQ;
  msg.data    = buffer;
  msg.length  = 0;

  return host_can_cfg.send(&msg);
}

