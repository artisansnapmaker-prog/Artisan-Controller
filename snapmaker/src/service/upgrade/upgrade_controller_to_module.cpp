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
#include "upgrade_module.h"
#include "upgrade_service.h"
#include "../../snapmaker.h"
#include "../../module/toolhead_laser.h"

UpgradeModuleInfo upgrade_module_info_tab[] = {
  
  // {
  //   ESP32_FW,                                             /* packet type */
  //   0,                                                    /* start id    */
  //   0,                                                    /* end id      */
  //   NULL,
  //   NULL,
  //   {
  //     esp32_camera_upgrade_start, 
  //     module_call_start_ack, 
  //     module_call_trans_req, 
  //     esp32_camera_upgrade_trans, 
  //     esp32_camera_upgrade_end, 
  //     module_call_end_ack, 
  //     module_call_notify_req
  //     NULL, 
  //   }
  // },

  // {
  //   SM2_MODULE_FW,                                        /* packet type */
  //   0,                                                    /* start id    */
  //   13,                                                   /* end id      */
  //   NULL,
  //   NULL,
  //   {
  //     NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
  //   }
  // },
  
};

err_code_t module_call_start_ack(uint8_t ret) {
  FUN_LOG();
  return ugr_mdl_svc.module_call_start_ack(ret);
}

err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len) {
  FUN_LOG();
  return ugr_mdl_svc.module_call_trans_req(req_offset, len);
}

err_code_t module_call_end_ack(uint8_t ret) {
  FUN_LOG();
  return ugr_mdl_svc.module_call_end_ack(ret);
}

err_code_t module_call_notify_req(uint8_t ret) {
  FUN_LOG();
  return ugr_mdl_svc.module_call_notify_req(ret);
}

err_code_t UpgradeModuleService::init(UpdateService *s) {
  status = UPGRADE_MODULE_STATUS_IDLE;
  return E_SUCCESS;
}

void UpgradeModuleService::loop(void) {

  switch(status) {
    case UPGRADE_MODULE_STATUS_IDLE:
    break;

    case UPGRADE_MODULE_STATUS_START:
      if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
        req_trans_data(offset, UPGRADE_TRANS_BUF_SIZE);
        status = UPGRADE_MODULE_STATUS_TRANS;
      }
      else {
        if (time_after(millis(), last_start_req_ms + 500)) {
          if (start_req_try < 10) {
            start_req_try++;
          }
          else {
            LOG_E("upgrade_module: upgrade start request timeout, return to upgrade init\r\n");
            reset_to_idle();
          }
        }
      }
    break;

    case UPGRADE_MODULE_STATUS_TRANS:
      if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
        if (!trans_req_try || time_after(millis(), last_trans_req_ms + 500)) {
          if (trans_req_try < 10) {
            req_trans_data(offset, UPGRADE_TRANS_BUF_SIZE);
          }
          else {
            LOG_E("upgrade_module: upgrade trans timeout, return to upgrade init\r\n");
            reset_to_idle();
          }
        }
      }
      else {
        if (time_after(millis(), last_trans_req_ms + 500)) {
          if (trans_req_try < 10) {
            trans_req_try++;
          }
          else {
            LOG_E("upgrade_module: upgrade trans timeout, return to upgrade init\r\n");
            reset_to_idle();
          }
        }
      }
    break;

    case UPGRADE_MODULE_STATUS_END:
      if (time_after(millis(), last_end_req_ms + 500)) {
        if (end_req_try < 10) {
          // UPGRADE_MODULE_BY_HOST and UPGRADE_MODULE_BY_CONTROLLER
          // use the same api to notify the HOST
          notify_end(end_ret);
        }
        else {
          LOG_E("upgrade_module: upgrade end error, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    default:
    break;
  }
}

void UpgradeModuleService::reset_to_idle(void) {
  if (module_upgrade_info) {
    module_upgrade_info->handle.module_deinit();
  }
  status = UPGRADE_MODULE_STATUS_IDLE;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_INIT);
  smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
}

err_code_t UpgradeModuleService::start_proc(pack_info_t *bti, sacp_hmi_message_t *msg) {

  if (UPGRADE_MODULE_STATUS_IDLE != status) {
    LOG_E("upgrade_module: can not start a upgrade as current is not in IDLE status\r\n");
    return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
  }

  ugr_type = module_upgrade_type(bti);
  if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
    if (!flash_erase(module_fw_partition)) {
      if (!flash_erase(module_fw_partition)) {
        LOG_E("upgrade_module: module flash partition erase failure\r\n");
        return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
      }
    }
  }
  else {
    module_upgrade_info = get_module_upgrade_handls((UpdatePackType)bti->pack_type, bti->start_index);
    if (!module_upgrade_info) {
      LOG_E("upgrade_module: unsupported pack %d with id %d\r\n", bti->pack_type, bti->start_index);
      return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
    }

    if (E_SUCCESS != module_upgrade_info->module_init(&(module_upgrade_info->handle))) {
      LOG_E("upgrade_module: module upgrade init error\r\n");
      return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
    }
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
    LOG_E("upgrade_module: can not enter module upgrade status\r\n");
    reset_to_idle();
    return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
  }

  host_id = msg->peer;
  host_ch = msg->ch;
  offset = 0;
  fw_lenght = bti->fw_lenght;
  checksum = bti->fw_checksum;
  status = UPGRADE_MODULE_STATUS_START;
  if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
    ugr_svc->set_updgrade_phase(UPGRADE_PHASE_HOST_TO_CONTROLLER);
    ugr_svc->upgrade_start_ack(msg, E_SUCCESS);
  }
  else {
    ugr_svc->set_updgrade_phase(UPGRADE_PHASE_HOST_TO_MODULE);
    module_upgrade_info->handle.start_req(bti);
    last_start_req_ms = millis();
    start_req_try = 0;
  }

  return E_SUCCESS;
}

err_code_t UpgradeModuleService::trans_proc(pack_info_t *pack_info_t, sacp_hmi_message_t *msg) {
  uint32_t rx_offset;
  uint16_t rx_pack_len;

  if (UPGRADE_MODULE_STATUS_TRANS != status) {
    LOG_E("upgrade_module: can not handle trans packet as current is not in UPGRADE_STATE_TRANS state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (msg->length < 7) {
    LOG_E("upgrade_module: lenght error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (E_SUCCESS != msg->data[0]) {
    LOG_E("upgrade_module: trans return error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  rx_offset = LITTLE_STREAM_TO_32(msg->data+1);
  rx_pack_len = LITTLE_STREAM_TO_16(msg->data+5);
  configASSERT(rx_pack_len <= UPGRADE_TRANS_BUF_SIZE);
  LOG_I(">>> offset %d, pack_len %d\r\n", rx_offset, rx_pack_len);

  if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
    if (offset != rx_offset) {
      LOG_E("upgrade_module: offset not match\r\n");
      return ugr_svc->upgrade_notify(msg, E_FAILURE);
    }

    offset += flash_write(module_fw_partition, msg->data + 7, rx_pack_len);
    if (rx_pack_len) {
      trans_req_try = 0;
    }
  }
  else {
    configASSERT(module_upgrade_info);
    if(E_SUCCESS == module_upgrade_info->handle.trans_ack(rx_offset, msg->data + 7, rx_pack_len)) {
      offset += rx_pack_len;
    }
  }
  
  if (offset >= fw_lenght) {
    LOG_I("upgrade_module: RX ALL DATA\r\n");
    if (UPGRADE_MODULE_BY_CONTROLLER == ugr_type) {
      if (firmware_flash_checksum(checksum, module_fw_partition.start_addr, fw_lenght)) {
        end_ret = E_SUCCESS;
      }
      else {
        end_ret = E_FAILURE;
        LOG_E("upgrade_moduel: module firmware checksum failure in flash\r\n");
      }
    }
    else {
      configASSERT(module_upgrade_info);
      end_ret = E_SUCCESS;
      module_upgrade_info->handle.end_req(end_ret);
    }
    status = UPGRADE_MODULE_STATUS_END;
  }

  return E_SUCCESS;
}

err_code_t UpgradeModuleService::end_proc(pack_info_t *pack_info_t, sacp_hmi_message_t *msg) {

  if (UPGRADE_MODULE_STATUS_END != status) {
    LOG_E("upgrade_module: can not handle end ack packet as current is not in UPGRADE_MODULE_STATUS_END state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (!msg->length || E_SUCCESS != msg->data[0]) {
    return E_SUCCESS;
  }

  reset_to_idle();
  return E_SUCCESS;
}

err_code_t UpgradeModuleService::module_call_start_ack(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  uint8_t data[4];
  sacp_hmi_message_t msg;
  msg.ver = SACP_VER_1;
  msg.peer = host_id;
  msg.ch = host_ch;
  msg.attr = 0;
  msg.cmd_set = CMD_SET_UPGRADE;
  msg.cmd_id = CMD_ID_UPGRADE_START;
  msg.data = data;
  msg.data[0] = ret;
  msg.length = 1;
  return ugr_svc->upgrade_start_ack(&msg, ret);
}

err_code_t UpgradeModuleService::module_call_trans_req(uint32_t req_offset, uint32_t len) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  if (UPGRADE_MODULE_STATUS_START == status) {
    status = UPGRADE_MODULE_STATUS_TRANS;
  }

  trans_req_try = 0;
  req_trans_data(req_offset, len);
  return E_SUCCESS;
}

err_code_t UpgradeModuleService::module_call_end_ack(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  notify_end(ret);
  return E_SUCCESS;
}

err_code_t UpgradeModuleService::module_call_notify_req(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  notify_error(ret);
  return E_SUCCESS;
}

void UpgradeModuleService::req_trans_data(uint32_t offset, uint16_t len) {
  uint32_t index = 0;
  sacp_hmi_message_t tx_msg;
  uint8_t send_frame[16];

  tx_msg.ver = SACP_VER_1;
  tx_msg.peer = host_id;
  tx_msg.ch = host_ch;
  tx_msg.attr = 0;
  tx_msg.cmd_set = CMD_SET_UPGRADE;
  tx_msg.cmd_id = CMD_ID_UPGRADE_TRANS;
  tx_msg.data = send_frame;
  _32_TO_LITTLE_STREAM(offset, tx_msg.data + index);
  index += 4;
  _16_TO_LITTLE_STREAM(len, tx_msg.data + index);
  index += 2;
  tx_msg.length = index;
  LOG_I("%dms upgrade_module: trans_req offset %d, buffer %d\r\n", millis(), offset, len);
  host_hmi.send(&tx_msg);

  last_trans_req_ms = millis();
  trans_req_try++;
}

void UpgradeModuleService::notify_end(uint8_t ret) {
  sacp_hmi_message_t tx_msg;
  uint8_t send_frame[4];

  tx_msg.ver = SACP_VER_1;
  tx_msg.peer = host_id;
  tx_msg.ch = host_ch;
  tx_msg.attr = 0;
  tx_msg.cmd_set = CMD_SET_UPGRADE;
  tx_msg.cmd_id = CMD_ID_UPGRADE_END;
  tx_msg.data = send_frame;
  tx_msg.data[0] = ret;
  tx_msg.length = 1;
  host_hmi.send(&tx_msg);
  LOG_I("%dms upgrade_module: end_req, ret %d\r\n", millis(), ret);

  last_end_req_ms = millis();
  end_req_try++;
}

void UpgradeModuleService::notify_error(uint8_t ret) {
  sacp_hmi_message_t tx_msg;
  uint8_t send_frame[4];

  tx_msg.ver = SACP_VER_1;
  tx_msg.peer = host_id;
  tx_msg.ch = host_ch;
  tx_msg.attr = 0;
  tx_msg.cmd_set = CMD_SET_UPGRADE;
  tx_msg.cmd_id = CMD_ID_UPGRADE_NOTIFY;
  tx_msg.data = send_frame;
  tx_msg.data[0] = ret;
  tx_msg.length = 1;
  host_hmi.send(&tx_msg);
  LOG_I("%dms upgrade_module: notify %d\r\n", millis(), ret);
}

bool UpgradeModuleService::firmware_flash_checksum(uint32_t rx_checsum, uint32_t flash_addr, uint32_t len) {
  uint32_t cs;
  cs = calculate_checksum((uint8_t *)flash_addr, len);
  return cs == rx_checsum;
}

ModuleUpgradeType UpgradeModuleService::module_upgrade_type(pack_info_t *pack_info_t) {

  if (ESP32_FW == pack_info_t->pack_type) {
    return UPGRADE_MODULE_BY_HOST;
  }
  else if (SM2_MODULE_FW == pack_info_t->pack_type) {
    if (pack_info_t->fw_lenght > module_fw_partition.size) {
      LOG_I("upgrade_moduel: firmware too large for controller's flash. So, use traspare upgrade\r\n");
      return UPGRADE_MODULE_BY_HOST;
    }
    else 
      return UPGRADE_MODULE_BY_CONTROLLER;
  }
  else {
    return UPGRADE_MODULE_BY_HOST;
  }

}

UpgradeModuleInfo *UpgradeModuleService::get_module_upgrade_handls(UpdatePackType pack_type, uint16_t id) {
  for (uint32_t i = 0; i < TAB_SIZE(upgrade_module_info_tab, UpgradeModuleInfo); i++) {

    if (pack_type != upgrade_module_info_tab[i].pack_type)
      continue;

    if (ESP32_FW == pack_type) {
      return &(upgrade_module_info_tab[i]);
    }
    else if (SM2_MODULE_FW == pack_type) {
      if (upgrade_module_info_tab[i].start_id <= id && 
          id <= upgrade_module_info_tab[i].end_id) {
        return &(upgrade_module_info_tab[i]);
      }
    }

  }

  return NULL;
}
