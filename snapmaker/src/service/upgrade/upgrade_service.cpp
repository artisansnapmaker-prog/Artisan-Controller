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
#include "../../common/debug.h"
#include "../../common/utility.h"
#include "../../host/sacp_module.h"
#include "../../common/flash.h"
#include "../../snapmaker.h"
#include "upgrade_service.h"

UpdateService upgrade_svc;

err_code_t UpdateService::init(void) {
  err_code_t ret;

  phase = UPGRADE_PHASE_INIT;
  seq = 0;

  ret = E_SUCCESS;
  ret |= host_hmi.apply_cmd_set_handle(CMD_SET_UPGRADE, 3);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_START, this, sacp_upgrade_start);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_TRANS, this, sacp_upgrade_trans, SACP_CB_ATTR_ACK);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_END, this, sacp_upgrade_end, SACP_CB_ATTR_ACK);

  ctrl_ugr = &ugr_ctrl_svc;
  module_ugr = &ugr_mdl_svc;
  ret |= ctrl_ugr->init(this);
  ret |= module_ugr->init(this);

  return ret;
}

void UpdateService::loop(void) {
  ctrl_ugr->loop();
  module_ugr->loop();
}

err_code_t UpdateService::sacp_upgrade_start(void *obj, sacp_hmi_message_t *msg) {
  
  LOG_I("sacp_upgrade_start\r\n");
  UpdateService &upgrade = *(UpdateService *)obj;
  pack_info_t *bti = (pack_info_t *)msg->data;

  if (UPGRADE_PHASE_INIT != upgrade.phase) {
    LOG_E("ugr_svc: can not start a upgrade as not in INIT phase\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }

  if (!boot_info_check(bti)) {
    LOG_E("boot info checksum failure\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }

  if (A400_CONTROLLER_FW == bti->pack_type) {
    return upgrade.ctrl_ugr->start_proc(bti, msg);
  }
  else if (SM2_MODULE_FW == bti->pack_type ||
            ESP32_FW == bti->pack_type
  ){
    return upgrade.module_ugr->start_proc(bti, msg);
  }
  else {
    LOG_E("upgrade_service: unsupported packet type\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }
  
  #if 0
  switch (bti->pack_type)
  {
  case A400_CONTROLLER_FW:
    break;
  
  case SM2_MODULE_FW:
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
      LOG_E("upgrade: can not enter upgrade status\r\n");
      return upgrade.upgrade_start_ack(msg, E_FAILURE);  
    }
    break;

  case ESP32_FW:
    LOG_I("upgrade: TODO: \r\n");
    break;

  case SM2_CONTROLLER_FW:
  case J1_CONTROLLER_FW:
  default:
    LOG_E("upgrade: unsupported packet type\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
    break;
  }

  if (SM2_MODULE_FW == bti->pack_type) {
    LOG_I("do module upgrade\r\n");
    if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
      LOG_E("upgrade: can not enter upgrade status\r\n");
      return upgrade.upgrade_start_ack(msg, E_FAILURE);  
    }
    
    return upgrade.upgrade_start_ack(msg, E_SUCCESS);
  }

  if (A400_CONTROLLER_FW != bti->pack_type) {
    LOG_E("not a400 controller pack\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }
  #endif
}

err_code_t UpdateService::sacp_upgrade_trans(void *obj, sacp_hmi_message_t *msg) {
  LOG_I("sacp_upgrade_trans\r\n");

  // Always module upgrade
  UpdateService &upgrade = *(UpdateService *)obj;
  return upgrade.module_ugr->trans_proc(&(upgrade.boot_info), msg);
}

err_code_t UpdateService::sacp_upgrade_end(void *obj, sacp_hmi_message_t *msg) {
  LOG_I("sacp_upgrade_end\r\n");

  // Always module upgrade
  UpdateService &upgrade = *(UpdateService *)obj;
  return upgrade.module_ugr->end_proc(&(upgrade.boot_info), msg);
}

err_code_t UpdateService::upgrade_start_ack(sacp_hmi_message_t *msg, err_code_t ret) {
  return host_hmi.send_ack(msg, ret);
}

err_code_t UpdateService::upgrade_notify(sacp_hmi_message_t *msg, err_code_t ret) {
  uint8_t send_buf[8];

  msg->attr = 0;
  msg->seq = seq++;
  msg->cmd_id = CMD_SET_UPGRADE;
  msg->cmd_id = CMD_ID_UPGRADE_NOTIFY;
  msg->data = send_buf;
  msg->data[0] = ret;
  msg->length = 1;
  return host_hmi.send(msg);
}

bool UpdateService::boot_info_flush_to_flash() {
  boot_info.boot_data_checksum = calculate_checksum((uint8_t *)&boot_info, sizeof(pack_info_t) - 4);
  if (!flash_erase(boot_data_partition)) {
    LOG_E("boot data erase error\r\n");
    return false;
  }

  if (sizeof(pack_info_t) != flash_write(boot_data_partition, (uint8_t *)&boot_info, sizeof(pack_info_t))) {
    LOG_E("boot data write error\r\n");
    return false;
  }

  return true;
}

void UpdateService::set_boot_info(pack_info_t *bti) {
  boot_info = *bti;
}

void UpdateService::print_boot_info(void) {
  LOG_I("========== boot info ==========\r\n");
  LOG_I("magic_str: %s\r\n", (char *)boot_info.magic_str);
  LOG_I("protocol ver: %d\r\n", boot_info.protocol_ver);
  LOG_I("pack_type: %d\r\n", boot_info.pack_type);
  LOG_I("upgrade ctrl flag: %d\r\n", boot_info.upgrade_ctrl_flag);
  LOG_I("start index: %d\r\n", boot_info.start_index);
  LOG_I("end index: %d\r\n", boot_info.end_index);
  LOG_I("fw version: %s\r\n", boot_info.fw_ver_str);
  LOG_I("timestamp: %s\r\n", boot_info.timestamp_str);
  LOG_I("upgrade state: 0x%04x\r\n", boot_info.upgrade_state);
  LOG_I("fw lenght: %d, 0x%04x\r\n", boot_info.fw_lenght, boot_info.fw_lenght);
  LOG_I("fw checksum: 0x%08x\r\n", boot_info.fw_checksum);
  LOG_I("fw run addr: 0x%08x\r\n", boot_info.fw_runaddr);
  LOG_I("peer: %d\r\n", boot_info.peer);
  LOG_I("link_ch: %d\r\n", boot_info.link_ch);
  LOG_I("boot data checksum: 0x%08x", boot_info.boot_data_checksum);
}

uint32_t UpdateService::get_seq(void) {
  return seq++;
}

void UpdateService::set_updgrade_phase(UpgradePhase p) {
  phase = p;
}

void UpdateService::host_to_controller_loop(void) {

}
