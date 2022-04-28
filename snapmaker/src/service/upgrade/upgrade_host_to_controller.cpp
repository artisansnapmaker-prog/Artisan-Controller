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
#include "upgrade_host_to_controller.h"
#include "upgrade_service.h"
#include "../../snapmaker.h"

UpgradeHostToController ugr_hc_svc;

err_code_t UpgradeHostToController::init(UpdateService *s) {
  ugr_svc = s;
  status = UPGRADE_HC_STATUS_IDLE;
  return E_SUCCESS;
}

void UpgradeHostToController::loop(void) {

  switch(status) {
    case UPGRADE_HC_STATUS_IDLE:
    break;

    case UPGRADE_HC_STATUS_START:
      last_action_ms = millis();
      action_retry++;
      trans_data_req(offset, UPGRADE_TRANS_BUF_SIZE);
      status = UPGRADE_HC_STATUS_TRANS;
    break;

    case UPGRADE_HC_STATUS_TRANS:
      if (time_after(millis(), last_action_ms + 500)) {
        if (action_retry < 10) {
          last_action_ms = millis();
          action_retry++;
          trans_data_req(offset, UPGRADE_TRANS_BUF_SIZE);
        }
        else {
          LOG_E("upgrade_hc: upgrade trans timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_HC_STATUS_START_CM:
      if (E_SUCCESS != ugr_cm_svc.start()) {
        LOG_E("upgrade_hc: start CONTROLLER to MODULE failure\r\n");
        end_ret = E_FAILURE;
        status = UPGRADE_HC_STATUS_END;
        last_action_ms = millis();
        action_retry++;
        end_req(end_ret);
      }
      else {
        status = UPGRADE_HC_STATUS_DO_CM;
      }
    break;

    case UPGRADE_HC_STATUS_DO_CM:
      if (UPGRADE_CM_STATUS_IDLE == ugr_cm_svc.get_status()) {
        end_ret = E_SUCCESS;
        status = UPGRADE_HC_STATUS_END;
        last_action_ms = millis();
        action_retry++;
        end_req(end_ret);
      }
    break;

    case UPGRADE_HC_STATUS_END:
      if (time_after(millis(), last_action_ms + 500)) {
        if (action_retry < 10) {
          last_action_ms = millis();
          action_retry++;
          end_req(end_ret);
        }
        else {
          LOG_E("upgrade_hc: upgrade end error, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    default:
    break;
  }
}

void UpgradeHostToController::reset_to_idle(void) {
  status = UPGRADE_HC_STATUS_IDLE;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_INIT);
  smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
}

err_code_t UpgradeHostToController::sacp_msg_proc(sacp_hmi_message_t *msg) {  
  switch(msg->cmd_id) {
    case CMD_ID_UPGRADE_START:
      return start_proc(msg);
    break;

    case CMD_ID_UPGRADE_TRANS:
      return trans_proc(msg);
    break;

    case CMD_ID_UPGRADE_END:
      return end_proc(msg);
    break;

    case CMD_ID_UPGRADE_NOTIFY:
      return E_SUCCESS;
    break;

    default:
      return E_FAILURE;
    break;
  }
}

err_code_t UpgradeHostToController::start_proc(sacp_hmi_message_t *msg) {
  pack_info_t *pit;

  pit = (pack_info_t *)(msg->data+2);
  if (!boot_info_check(pit)) {
    LOG_E("upgrade_hc: packet info checksum failure\r\n");
    return start_ack(msg, E_FAILURE);
  }

  if (UPGRADE_HC_STATUS_IDLE != status) {
    LOG_E("upgrade_hc: can not start a upgrade as current is not in IDLE status\r\n");
    return start_ack(msg, E_FAILURE);
  }

  if (!flash_erase(module_fw_partition)) {
    if (!flash_erase(module_fw_partition)) {
      LOG_E("upgrade_hc: module flash partition erase failure\r\n");
      return start_ack(msg, E_FAILURE);
    }
  }

  if (BOOT_INFO_SIZE != flash_write(module_fw_partition, (uint8_t *)pit, BOOT_INFO_SIZE)) {
    if (BOOT_INFO_SIZE != flash_write(module_fw_partition, (uint8_t *)pit, BOOT_INFO_SIZE)) {
      LOG_E("upgrade_hc: can not write packet info\r\n");
      return start_ack(msg, E_FAILURE);
    }
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
    LOG_E("upgrade_hc: can not enter module upgrade status\r\n");
    return start_ack(msg, E_FAILURE);
  }
  ugr_svc->print_packet_info(pit);

  host_id = msg->peer;
  host_ch = msg->ch;
  offset = 0;
  fw_lenght = pit->fw_lenght;
  checksum = pit->fw_checksum;
  status = UPGRADE_HC_STATUS_START;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_HOST_TO_CONTROLLER);
  start_ack(msg, E_SUCCESS);

  return E_SUCCESS;
}

err_code_t UpgradeHostToController::trans_proc(sacp_hmi_message_t *msg) {
  uint32_t rx_offset;
  uint16_t rx_pack_len;

  if (UPGRADE_HC_STATUS_TRANS != status) {
    LOG_E("upgrade_hc: can not handle trans packet as current is not in UPGRADE_STATE_TRANS state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (msg->length < 7) {
    LOG_E("upgrade_hc: lenght error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (E_SUCCESS != msg->data[0]) {
    LOG_E("upgrade_hc: trans return error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  rx_offset = LITTLE_STREAM_TO_32(msg->data+1);
  rx_pack_len = LITTLE_STREAM_TO_16(msg->data+5);
  configASSERT(rx_pack_len <= UPGRADE_TRANS_BUF_SIZE);
  LOG_I(">>> offset %d, pack_len %d\r\n", rx_offset, rx_pack_len);
  
  if (offset != rx_offset) {
    LOG_E("upgrade_hc: offset not match\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  offset += flash_write(module_fw_partition, msg->data + 7, rx_pack_len);
  if (rx_pack_len) {
    action_retry = 0;
  }
  last_action_ms = millis();
  action_retry++;

  if (offset >= fw_lenght) {
    LOG_I("upgrade_hc: RX ALL DATA\r\n");
    if (ugr_svc->firmware_flash_checksum(checksum, module_fw_partition.start_addr + BOOT_INFO_SIZE, fw_lenght)) {
      end_ret = E_SUCCESS;
      status = UPGRADE_HC_STATUS_START_CM;
    }
    else {
      end_ret = E_FAILURE;
      status = UPGRADE_HC_STATUS_END;
      last_action_ms = millis();
      action_retry++;
      LOG_E("upgrade_moduel: module firmware checksum failure in flash\r\n");
    }
  }
  else {
    trans_data_req(offset, UPGRADE_TRANS_BUF_SIZE);
  }

  return E_SUCCESS;
}

err_code_t UpgradeHostToController::end_proc(sacp_hmi_message_t *msg) {

  if (UPGRADE_HC_STATUS_END != status) {
    LOG_E("upgrade_hc: can not handle end ack packet as current is not in UPGRADE_HC_STATUS_END state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (!msg->length) {
    return E_SUCCESS;
  }

  reset_to_idle();
  return E_SUCCESS;
}

err_code_t UpgradeHostToController::start_ack(sacp_hmi_message_t *msg, uint8_t ret) {
  return host_hmi.send_ack(msg, ret);
}

void UpgradeHostToController::trans_data_req(uint32_t offset, uint16_t len) {
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
  LOG_I("%dms upgrade_hc: trans_req offset %d, buffer %d\r\n", millis(), offset, len);
  host_hmi.send(&tx_msg);
}

void UpgradeHostToController::end_req(uint8_t ret) {
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
  LOG_I("%dms upgrade_hc: end_req, ret %d\r\n", millis(), ret);
}

void UpgradeHostToController::error_notify(uint8_t ret) {
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
  LOG_I("%dms upgrade_hc: notify %d\r\n", millis(), ret);
}
