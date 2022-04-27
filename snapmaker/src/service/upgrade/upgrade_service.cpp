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

  update_flash_mark();
  phase = UPGRADE_PHASE_INIT;

  ret = E_SUCCESS;
  ret |= host_hmi.apply_cmd_set_handle(CMD_SET_UPGRADE, 3);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_START, this, sacp_msg_proc);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_TRANS, this, sacp_msg_proc, SACP_CB_ATTR_ACK);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_END, this, sacp_msg_proc, SACP_CB_ATTR_ACK);

  ret |= ugr_ctrl_svc.init(this);
  ret |= ugr_hc_svc.init(this);
  ret |= ugr_cm_svc.init(this);

  return ret;
}

void UpdateService::update_flash_mark(void) {
  memcpy(&boot_info, (void *)FLASH_BOOT_DATA_ADDR, sizeof(boot_info));
  boot_info.upgrade_state = UPGRADE_STATE_JUMP_SUCCESS;
  boot_info_flush_to_flash();
  LOG_I("ugr_svc: mark boot data as UPGRADE_STATE_JUMP_SUCCESS\r\n");
}

void UpdateService::loop(void) {
  ugr_ctrl_svc.loop();
  ugr_hc_svc.loop();
  ugr_hm_svc.loop();
  ugr_cm_svc.loop();
}

err_code_t UpdateService::sacp_msg_proc(void * obj, sacp_hmi_message_t *msg) {
  pack_info_t *pit;
  UpdateService &upgrade = *(UpdateService *)obj;

  switch (upgrade.phase) {
    case UPGRADE_PHASE_INIT:
      if (CMD_ID_UPGRADE_START != msg->cmd_id) {
        LOG_E("upgrade_service: only upgrade start can been accepted IN UPGRADE_PHASE_INIT\r\n");
        break;
      }

      pit = (pack_info_t *)(msg->data + 2);
      if (!boot_info_check(pit)) {
        LOG_E("upgrade_service: packet info checksum failure\r\n");
        break;
      }

      upgrade.phase = upgrade.upgrade_phase(pit);
      LOG_I("upgread_servicde: upgrade.phase %d\r\n", upgrade.phase);
      UpdateService::sacp_msg_proc(obj, msg);
    break;

    case UPGRADE_PAHSE_APP_START:
      pit = (pack_info_t *)(msg->data + 2);
      ugr_ctrl_svc.start_proc(pit, msg);
    break;

    case UPGRADE_PHASE_HOST_TO_CONTROLLER:
      ugr_hc_svc.sacp_msg_proc(msg);
    break;  

    case UPGRADE_PHASE_HOST_TO_MODULE:
      ugr_hm_svc.sacp_msg_proc(msg);
    break;

    case UPGRADE_PHASE_CONTROLLER_TO_MODULE:
    break;

    default:
      upgrade.phase = UPGRADE_PHASE_INIT;
    break;  
  }

  return E_SUCCESS;
}

UpgradePhase UpdateService::upgrade_phase(pack_info_t *pit) {
  if (A400_CONTROLLER_FW == pit->pack_type) {
    return UPGRADE_PAHSE_APP_START;
  }
  else if (ESP32_FW == pit->pack_type){
    return UPGRADE_PHASE_HOST_TO_MODULE;
  }
  else if (SM2_MODULE_FW == pit->pack_type) {
    if (pit->fw_lenght > module_fw_partition.size) {
      LOG_I("upgrade_service: firmware too large for controller's flash. So, use HOST TO MODULE upgrade\r\n");
      return UPGRADE_PHASE_HOST_TO_MODULE;
    }
    else
      return UPGRADE_PHASE_HOST_TO_CONTROLLER;
  }
  else {
    return UPGRADE_PHASE_INIT;
  }
}

err_code_t UpdateService::upgrade_start_ack(sacp_hmi_message_t *msg, err_code_t ret) {
  return host_hmi.send_ack(msg, ret);
}

err_code_t UpdateService::upgrade_notify(sacp_hmi_message_t *msg, err_code_t ret) {
  uint8_t send_buf[8];

  msg->attr = 0;
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
    if (!flash_erase(boot_data_partition)) {
      LOG_E("boot data erase error\r\n");
      return false;
    }
  }

  if (sizeof(pack_info_t) != flash_write(boot_data_partition, (uint8_t *)&boot_info, sizeof(pack_info_t))) {
    boot_data_partition.write_addr = boot_data_partition.start_addr;
    if (sizeof(pack_info_t) != flash_write(boot_data_partition, (uint8_t *)&boot_info, sizeof(pack_info_t))) {
      LOG_E("boot data write error\r\n");
      return false; 
    }
  }

  return true;
}

void UpdateService::set_boot_info(pack_info_t *bti) {
  boot_info = *bti;
}

void UpdateService::print_packet_info(pack_info_t *pit) {
  LOG_I("========== packet info ==========\r\n");
  LOG_I("magic_str: %s\r\n", (char *)pit->magic_str);
  LOG_I("protocol ver: %d\r\n", pit->protocol_ver);
  LOG_I("pack_type: %d\r\n", pit->pack_type);
  LOG_I("upgrade ctrl flag: %d\r\n", pit->upgrade_ctrl_flag);
  LOG_I("start index: %d\r\n", pit->start_index);
  LOG_I("end index: %d\r\n", pit->end_index);
  LOG_I("fw version: %s\r\n", pit->fw_ver_str);
  LOG_I("timestamp: %s\r\n", pit->timestamp_str);
  LOG_I("upgrade state: 0x%04x\r\n", pit->upgrade_state);
  LOG_I("fw lenght: %d, 0x%04x\r\n", pit->fw_lenght, pit->fw_lenght);
  LOG_I("fw checksum: 0x%08x\r\n", pit->fw_checksum);
  LOG_I("fw run addr: 0x%08x\r\n", pit->fw_runaddr);
  LOG_I("peer: %d\r\n", pit->peer);
  LOG_I("link_ch: %d\r\n", pit->link_ch);
  LOG_I("boot data checksum: 0x%08x", pit->boot_data_checksum);
}

UpgradePhase UpdateService::set_updgrade_phase(UpgradePhase p) {
  UpgradePhase lp = phase;
  phase = p;
  return lp;
}

UpgradePhase UpdateService::get_upgrade_pahse(void) {
  return phase;
}

bool UpdateService::firmware_flash_checksum(uint32_t rx_checsum, uint32_t flash_addr, uint32_t len) {
  uint32_t cs;
  cs = calculate_checksum((uint8_t *)flash_addr, len);
  return cs == rx_checsum;
}