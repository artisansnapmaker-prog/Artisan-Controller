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
#include "../../boot/boot_upgrade.h"
#include "esp32_upgrade.h"
#include "upgrade_module_interface.h"

static UpgradeModuleHandle *esp32_func_tab = NULL;
static uint32_t send_pack_index = ESP32_FW_PACK_INDEX_INVALID;
static uint32_t fw_file_offset = ESP32_FW_FILE_OFFSET_INVALID;

err_code_t esp32_camera_upgrade_handle_init(UpgradeModuleHandle *func_tab) {
  if (!func_tab) {
    LOG_E("[%s] func_tab is null\n", __FUNCTION__);
    return E_PARAM;
  }

  taskENTER_CRITICAL();
  func_tab->start_req = esp32_camera_upgrade_start;
  func_tab->trans_ack = esp32_camera_upgrade_trans;
  func_tab->end_req = esp32_camera_upgrade_end;
  esp32_func_tab = func_tab;
  taskEXIT_CRITICAL();

  return E_SUCCESS;
}

void esp32_camera_upgrade_handle_deinit(void) {
  LOG_I("%s\n",__FUNCTION__);
  taskENTER_CRITICAL();
  esp32_func_tab = NULL;
  send_pack_index = ESP32_FW_PACK_INDEX_INVALID;
  fw_file_offset = ESP32_FW_FILE_OFFSET_INVALID;
  taskEXIT_CRITICAL();
}

err_code_t esp32_camera_upgrade_start(pack_info_t *pit, module_info_t *mit) {
  err_code_t ret;
  sacp_hmi_message_t msg;
  ModuleBase *laser = NULL;
  uint8_t ver_buf[50];
  uint8_t data = 0;
  char *ptr = NULL;
  uint16_t recv_len = sizeof(ver_buf);
  uint16_t device_id = 0;
  uint16_t upgrade_id = 0;


  if (esp32_func_tab && esp32_func_tab->start_ack) {
    //  ESP32 directly replies that the reception is successful, and the upgrade judgment will be processed later.
    esp32_func_tab->start_ack(E_SUCCESS);
  }

  if (smprinter.get_toolhead_type() != TH_TYPE_LASER) {
    LOG_W("[%s] cur toolhead is not laser, no need to upgrade esp32\n", __FUNCTION__);
    return RET_SKIP_UPGRADE;
  }

  laser = smprinter.get_cur_toolhead();
  if (!laser) {
    LOG_W("[%s] failed to get toolhead, upgrading esp32 stop\n", __FUNCTION__);
    return RET_SKIP_UPGRADE;
  }

  device_id = laser->get_device_id();
  if (device_id == MODULE_DEVICE_ID_LASER_1P6W_2019) {
    if (pit->pack_type != ESP32_FW || pit->start_index != MODULE_DEVICE_ID_ESP32_NO_PARSM_OV2640 || \
        pit->end_index != MODULE_DEVICE_ID_ESP32_NO_PARSM_OV2640) {
      LOG_W("[%s] 1.6w laser no need to upgrade this version\n", __FUNCTION__);
      return RET_SKIP_UPGRADE;
    }
  }
  else if (device_id == MODULE_DEVICE_ID_LASER_10W_2021) {
    if (pit->pack_type != ESP32_FW || pit->start_index != MODULE_DEVICE_ID_ESP32_PARSM_OV2640 || \
        pit->end_index != MODULE_DEVICE_ID_ESP32_PARSM_OV2640) {
      LOG_W("[%s] 10w laser no need to upgrade this version\n", __FUNCTION__);
      return RET_SKIP_UPGRADE;
    }
  }
  else {
    LOG_W("[%s] unsupported upgrade type, device_id: %d\n", __FUNCTION__, device_id);
    return RET_SKIP_UPGRADE;
  }

  msg.attr = 0;
  msg.ch = SACP_HMI_CH_CAMERA;
  msg.cmd_set = M_REPORT_VERSIONS;
  msg.cmd_id = 0XFFFF;
  msg.peer = SACP_HOST_ID_ESP32;
  msg.ver = SACP_VER_0;
  msg.length = 1;
  msg.data = &data;

  memset(ver_buf, 0, recv_len);
  ret = host_hmi.send_sync_legacy(&msg, ver_buf, &recv_len, 100, 2);
  if (ret != E_SUCCESS || recv_len < ESP32_SSTP_MIN_VERION_MSG_LEN) {
    LOG_E("recv_len: %d failed to get BT version, upgrade not allowed. ret[%u]\n", recv_len, ret);
    return RET_SKIP_UPGRADE;
  }

  // More rules can be added here for version detection,
  // For example, add match determination for the queried esp32 version
  LOG_I("[%s] BT version: %s, recv_len = %d ret[%u]\n", __FUNCTION__, ver_buf+2, recv_len, ret);
  ptr = strrchr((char*)(pit->fw_ver_str), '-');
  if (ptr && strlen(ptr) == 5) {
    upgrade_id = (uint16_t)strtol(ptr+1, NULL, 16);
  }
  else {
    LOG_E("[%s] unsupported upgrade version\n", __FUNCTION__);
    return RET_SKIP_UPGRADE;
  }

  if (device_id == MODULE_DEVICE_ID_LASER_1P6W_2019) {
    if (upgrade_id != MODULE_DEVICE_ID_ESP32_NO_PARSM_OV2640) {
      LOG_E("[%s] mismatched device id\n", __FUNCTION__);
      return RET_SKIP_UPGRADE;
    }
  }
  else if (device_id == MODULE_DEVICE_ID_LASER_10W_2021) {
    if (strncmp((char*)(ver_buf+2), "V1.6", 4) == 0) {
      LOG_I("[%s] esp32 does not need to be upgraded\n", __FUNCTION__);
      return RET_SKIP_UPGRADE;
    }

    if (upgrade_id != MODULE_DEVICE_ID_ESP32_PARSM_OV2640) {
      LOG_E("[%s] unsupported upgrade version\n", __FUNCTION__);
      return RET_SKIP_UPGRADE;
    }
  }

  msg.attr = (SACP_MESSAGE_ATTR_SET_VER | SACP_MESSAGE_ATTR_SET_SEQ);
  msg.cmd_set = M_UPDATE_MOUDLE;
  msg.cmd_id = ESP32_UPDATE_OPCODE_START_NOTIFY;

  // initialise variables directly as soon as a request is made
  taskENTER_CRITICAL();
  send_pack_index = ESP32_FW_PACK_INDEX_INVALID;
  fw_file_offset = ESP32_FW_FILE_OFFSET_INVALID;
  taskEXIT_CRITICAL();

  if ((ret = host_hmi.send(&msg)) != E_SUCCESS) {
    LOG_E("[%s] failed to send upgrade start, ret[%u]\n", __FUNCTION__, ret);
  }

  return ret;
}

err_code_t esp32_camera_upgrade_trans(uint32_t offset, uint8_t *data, uint32_t len) {
  err_code_t ret;
  sacp_hmi_message_t msg;
  uint8_t *buffer = NULL;

  if (!data || !len) {
    LOG_E("[%s] invalid param, data is %s, len %d\n", __FUNCTION__, data ? "not null" : "null", len);
    return E_PARAM;
  }

  if (len > ESP32_FW_PACK_MAX_LEN) {
    LOG_E("[%s] len is too long\n", __FUNCTION__);
    return E_NO_MEM;
  }

  if (fw_file_offset != offset) {
    LOG_E("[%s] mismatched offset, need offset:%d get offset: %d\n",\
           __FUNCTION__, fw_file_offset, offset);
    return E_PARAM;
  }

  buffer = (uint8_t *)pvPortMalloc(sizeof(uint8_t) * len);
  if (!buffer) {
    LOG_E("[%s] buffer malloc fail\n", __FUNCTION__);
    return E_NO_MEM;
  }
  memcpy(buffer, data, len);
  msg.attr = (SACP_MESSAGE_ATTR_SET_VER | SACP_MESSAGE_ATTR_SET_SEQ);
  msg.ch = SACP_HMI_CH_CAMERA;
  msg.cmd_set = M_UPDATE_MOUDLE;
  msg.cmd_id = ESP32_UPDATE_OPCODE_TRANS_NOTIFY;
  msg.peer = SACP_HOST_ID_ESP32;
  msg.ver = SACP_VER_0;
  msg.length = len;
  msg.data = buffer;

  taskENTER_CRITICAL();
  fw_file_offset += len;
  taskEXIT_CRITICAL();

  if ((ret = host_hmi.send(&msg)) != E_SUCCESS) {
    LOG_E("[%s] failed to send upgrade start, ret[%u]\n", __FUNCTION__, ret);
  }

  if (buffer)
    vPortFree(buffer);

  return ret;
}

err_code_t esp32_camera_upgrade_end(uint8_t end_type) {
  err_code_t ret;
  sacp_hmi_message_t msg;

  if (end_type) {
    LOG_E("[%s] esp32_upgrade fail, end type: %d\n", __FUNCTION__, end_type);
    return E_FAILURE;
  }

  msg.attr = (SACP_MESSAGE_ATTR_SET_VER | SACP_MESSAGE_ATTR_SET_SEQ);
  msg.ch = SACP_HMI_CH_CAMERA;
  msg.cmd_set = M_UPDATE_MOUDLE;
  msg.cmd_id = ESP32_UPDATE_OPCODE_END_NOTIFY;
  msg.peer = SACP_HOST_ID_ESP32;
  msg.ver = SACP_VER_0;
  msg.length = 0;
  msg.data = NULL;
  if ((ret = host_hmi.send(&msg)) != E_SUCCESS) {
    LOG_E("[%s] failed to send upgrade start, ret[%u]\n", __FUNCTION__, ret);
  }
  return ret;
}


err_code_t esp32_camera_upgrade_start_ack_cb(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_FAILURE;
  // if (esp32_func_tab && esp32_func_tab->start_ack) {
  //   ret = E_SUCCESS;
  //   esp32_func_tab->start_ack(ret);
  // }
  return ret;
}

err_code_t esp32_camera_get_package_ack_cb(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_FAILURE;
  uint16_t get_pack_index = 0;
  bool index_err = true;
  if (!msg || !obj || msg->length != 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return E_PARAM;
  }

  get_pack_index = msg->data[0] << 8 | msg->data[1];
  LOG_I("[%s] get_pack_index: %d\n",__FUNCTION__, get_pack_index);
  if (send_pack_index == ESP32_FW_PACK_INDEX_INVALID) {
    if (get_pack_index != 0)
      index_err = false;
  }
  else {
    if (get_pack_index != send_pack_index + 1) {
      index_err = false;
    }
  }

  if (!index_err) {
    LOG_E("[%s] mismatched package index, get_pack_index:%d send_pack_index:%d\n",\
             __FUNCTION__, get_pack_index, send_pack_index);
  }
  else {
    taskENTER_CRITICAL();
    send_pack_index = get_pack_index;
    if (send_pack_index == 0) {
      fw_file_offset = 0;
    }
    taskEXIT_CRITICAL();

    if (esp32_func_tab && esp32_func_tab->trans_req) {
      esp32_func_tab->trans_req(fw_file_offset, ESP32_FW_PACK_MAX_LEN);
      ret = E_SUCCESS;
    }
    else {
      LOG_E("[%s] there is no corresponding function interface\n", __FUNCTION__);
    }
  }
  return ret;
}

err_code_t esp32_camera_updgrade_end_cb(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_FAILURE;
  if (esp32_func_tab && esp32_func_tab->end_ack) {
    ret = E_SUCCESS;
    esp32_func_tab->end_ack(ret);
  }
  return ret;
}

err_code_t esp32_camera_upgrade_fail_notify_cb(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_FAILURE;
  if (msg && msg->length == 2) {
    LOG_E("[%s] err_code: 0x%x", __FUNCTION__, msg->data[0] << 8 | msg->data[1]);
  }
  if (esp32_func_tab && esp32_func_tab->notify_req) {
    esp32_func_tab->notify_req(E_FAILURE);
    ret = E_SUCCESS;
  }
  return ret;
}
