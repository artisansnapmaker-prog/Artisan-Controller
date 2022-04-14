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

#include "upgrade_controller.h"
#include "upgrade_service.h"

err_code_t UpgradeCtrlService::init(UpdateService *s) {
  ugr_svc = s;
  return E_SUCCESS;
}

err_code_t UpgradeCtrlService::proc(boot_info_t *boot_info, sacp_hmi_message_t *msg) {

  if (SACP_HMI_CH_SCREEN == msg->ch) {
    boot_info->link_ch = LINK_CH_SC;
  } else if(SACP_HMI_CH_PC == msg->ch) {
    boot_info->link_ch = LINK_CH_PC;
  }
  else {
    LOG_E("unsupport channal %d\r\n", msg->ch);
    return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
  }

  boot_info->peer = msg->peer;
  boot_info->upgrade_state = UPGRADE_STATE_START;
  ugr_svc->set_boot_info((boot_info_t *)(msg->data));

  if (!ugr_svc->boot_info_flush_to_flash()) {
    if (!ugr_svc->boot_info_flush_to_flash()) {
      LOG_E("can not write boot info to flash\r\n");
      return ugr_svc->upgrade_start_ack(msg, E_FAILURE);
    }
  }

  ugr_svc->print_boot_info();
  LOG_I("System will restart in 1 second to start updating\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));
  NVIC_SystemReset();

}