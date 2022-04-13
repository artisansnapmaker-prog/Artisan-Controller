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
#include "../common/utility.h"
#include "../host/sacp_module.h"
#include "../common/flash.h"
#include "upgrade.h"

UpdateService upgrade_svc;

err_code_t UpdateService::init(void) {
err_code_t ret;

  ret = E_SUCCESS;
  ret |= host_hmi.apply_cmd_set_handle(CMD_SET_UPGRADE, 1);
  ret |= host_hmi.register_callback(CMD_SET_UPGRADE, CMD_ID_UPGRADE_START, this, sacp_upgrade_start);

  load_boot_info(&boot_info);
  if (boot_info_check(&boot_info)) {
    if (UPGRADE_STATE_JUMP_SUCCESS != boot_info.upgrade_state) {
    boot_info.upgrade_state = UPGRADE_STATE_JUMP_SUCCESS;
      if (!boot_info_flush_to_flash()) {
        if (!boot_info_flush_to_flash()) {
          LOG_E("can not write boot info to flash\r\n");
        }
      }
    }
  }
  else {
    LOG_E("boot info check failure\r\n");
  }

  return ret;
}

err_code_t UpdateService::sacp_upgrade_start(void *obj, sacp_hmi_message_t *msg) {
  uint8_t send_buf[8];
  UpdateService &upgrade = *(UpdateService *)obj;

  LOG_I("sacp_upgrade_start\r\n");
  // if (msg->length < CMD_START_MIN_LEN) {
  //   LOG_E("upgrade start request len error, expected %d, but get %d\r\n", CMD_START_MIN_LEN, CMD_START_MIN_LEN);
  //   return upgrade.upgrade_start_ack(msg, E_FAILURE);
  // }

  boot_info_t *bti = (boot_info_t *)msg->data;
  if (!boot_info_check(bti)) {
    LOG_E("boot info checksum failure\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }

  if (SM2_MODULE_FW == bti->pack_type) {
    LOG_I("do module upgrade, TODO: \r\n");
    return upgrade.upgrade_start_ack(msg, E_SUCCESS);
  }

  if (A400_CONTROLLER_FW != bti->pack_type) {
    LOG_E("not a400 controller pack\r\n");
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }

  if (SACP_HMI_CH_SCREEN == msg->ch) {
    bti->link_ch = LINK_CH_SC;
  } else if(SACP_HMI_CH_PC == msg->ch) {
    bti->link_ch = LINK_CH_PC;
  }
  else {
    LOG_E("unsupport channal %d\r\n", msg->ch);
    return upgrade.upgrade_start_ack(msg, E_FAILURE);
  }
  bti->peer = msg->peer;
  bti->upgrade_state = UPGRADE_STATE_START;

  upgrade.set_boot_info((boot_info_t *)(msg->data));
  if (!upgrade.boot_info_flush_to_flash()) {
    if (!upgrade.boot_info_flush_to_flash()) {
      LOG_E("can not write boot info to flash\r\n");
      return upgrade.upgrade_start_ack(msg, E_FAILURE);
    }
  }
  
  upgrade.print_boot_info();

  LOG_I("System will restart in 1 second to start updating\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));

  NVIC_SystemReset();
  return E_SUCCESS;
}

err_code_t UpdateService::upgrade_start_ack(sacp_hmi_message_t *msg, err_code_t ret) {

  uint8_t send_buf[8];

  send_buf[0] = ret;
  msg->data = send_buf;
  msg->length = 1;
  host_hmi.send_ack(msg);
  return E_SUCCESS;
}

bool UpdateService::boot_info_flush_to_flash() {
  boot_info.boot_data_checksum = calculate_checksum((uint8_t *)&boot_info, sizeof(boot_info_t) - 4);
  if (!flash_erase(boot_data_partition)) {
    LOG_E("boot data erase error\r\n");
    return false;
  }

  if (sizeof(boot_info_t) != flash_write(boot_data_partition, (uint8_t *)&boot_info, sizeof(boot_info_t))) {
    LOG_E("boot data write error\r\n");
    return false;
  }

  return true;
}

void UpdateService::set_boot_info(boot_info_t *bti) {
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