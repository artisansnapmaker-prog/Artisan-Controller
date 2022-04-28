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
#include "upgrade_host_to_module.h"
#include "upgrade_service.h"
#include "../../snapmaker.h"
#include "upgrade_module_interface.h"


UpgradeHostToModule ugr_hm_svc;

err_code_t UpgradeHostToModule::init(UpdateService *s) {
  ugr_svc = s;
  status = UPGRADE_HM_STATUS_IDLE;
  return E_SUCCESS;
}

void UpgradeHostToModule::loop(void) {

  switch(status) {
    case UPGRADE_HM_STATUS_IDLE:
    break;

    case UPGRADE_HM_STATUS_START:
      if (time_after(millis(), last_start_req_ms + 500)) {
        if (start_req_try < 10) {
          start_req_try++;
          trans_req_try = 0;
          last_start_req_ms = millis();
        }
        else {
          LOG_E("upgrade_hm: upgrade start request timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_HM_STATUS_TRANS:
      if (time_after(millis(), last_trans_req_ms + 500)) {
        if (trans_req_try < 10) {
          trans_req_try++;
          last_trans_req_ms = millis();
        }
        else {
          LOG_E("upgrade_hm: upgrade trans timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_HM_STATUS_END:
      if (time_after(millis(), last_end_req_ms + 500)) {
        if (end_req_try < 10) {
          end_req_try++;
          last_end_req_ms = millis();
        }
        else {
          LOG_E("upgrade_hm: upgrade end error, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    default:
    break;
  }
}

void UpgradeHostToModule::reset_to_idle(void) {
  if (module_upgrade_info) {
    module_upgrade_info->module_deinit();
  }
  status = UPGRADE_HM_STATUS_IDLE;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_INIT);
  smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
}

err_code_t UpgradeHostToModule::sacp_msg_proc(sacp_hmi_message_t *msg) {
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

err_code_t UpgradeHostToModule::start_proc(sacp_hmi_message_t *msg) {
  pack_info_t *pit;

  pit = (pack_info_t *)(msg->data+2);
  if (!boot_info_check(pit)) {
    LOG_E("upgrade_hm: packet info checksum failure\r\n");
    return start_ack(msg, E_FAILURE);
  }

  if (UPGRADE_HM_STATUS_IDLE != status) {
    LOG_E("upgrade_hm: can not start a upgrade as current is not in IDLE status\r\n");
    return start_ack(msg, E_FAILURE);
  }

  module_upgrade_info = get_module_upgrade_handls((UpdatePackType)pit->pack_type, pit->start_index);
  if (!module_upgrade_info) {
    LOG_E("upgrade_hm: unsupported pack %d with id %d\r\n", pit->pack_type, pit->start_index);
    return start_ack(msg, E_FAILURE);
  }

  if (E_SUCCESS != module_upgrade_info->module_init(&(module_upgrade_info->handle))) {
    LOG_E("upgrade_hm: module upgrade init error\r\n");
    return start_ack(msg, E_FAILURE);
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
    LOG_E("upgrade_hm: can not enter module upgrade status\r\n");
    reset_to_idle();
    return start_ack(msg, E_FAILURE);
  }
  ugr_svc->print_packet_info(pit);

  host_id = msg->peer;
  host_ch = msg->ch;
  offset = 0;
  fw_lenght = pit->fw_lenght;
  checksum = pit->fw_checksum;
  status = UPGRADE_HM_STATUS_START;

  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_HOST_TO_MODULE);
  module_upgrade_info->handle.start_req(pit, NULL);
  last_start_req_ms = millis();
  start_req_try = 0;

  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::trans_proc(sacp_hmi_message_t *msg) {
  uint32_t rx_offset;
  uint16_t rx_pack_len;

  if (UPGRADE_HM_STATUS_TRANS != status) {
    LOG_E("upgrade_hm: can not handle trans packet as current is not in UPGRADE_STATE_TRANS state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (msg->length < 7) {
    LOG_E("upgrade_hm: lenght error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (E_SUCCESS != msg->data[0]) {
    LOG_E("upgrade_hm: trans return error\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  rx_offset = LITTLE_STREAM_TO_32(msg->data+1);
  rx_pack_len = LITTLE_STREAM_TO_16(msg->data+5);
  configASSERT(rx_pack_len <= UPGRADE_TRANS_BUF_SIZE);
  LOG_I(">>> offset %d, pack_len %d\r\n", rx_offset, rx_pack_len);

  configASSERT(module_upgrade_info);
  if(E_SUCCESS == module_upgrade_info->handle.trans_ack(rx_offset, msg->data + 7, rx_pack_len)) {
    offset += rx_pack_len;
  }
  
  if (offset >= fw_lenght) {
    LOG_I("upgrade_hm: RX all the data\r\n");
    configASSERT(module_upgrade_info);
    end_ret = E_SUCCESS;
    last_end_req_ms = millis();
    end_req_try = 0;
    module_upgrade_info->handle.end_req(end_ret);
    status = UPGRADE_HM_STATUS_END;
  }

  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::end_proc(sacp_hmi_message_t *msg) {

  if (UPGRADE_HM_STATUS_END != status) {
    LOG_E("upgrade_hm: can not handle end ack packet as current is not in UPGRADE_HM_STATUS_END state\r\n");
    return ugr_svc->upgrade_notify(msg, E_FAILURE);
  }

  if (!msg->length) {
    return E_SUCCESS;
  }

  reset_to_idle();
  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::module_call_start_ack(uint8_t ret) {
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

  last_start_req_ms = millis();
  start_req_try = 0;
  return start_ack(&msg, ret);
}

err_code_t UpgradeHostToModule::module_call_trans_req(uint32_t req_offset, uint32_t len) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  if (UPGRADE_HM_STATUS_START == status) {
    status = UPGRADE_HM_STATUS_TRANS;
  }

  if (UPGRADE_HM_STATUS_TRANS == status) {
    trans_req_try = 0;
    trans_data_req(req_offset, len);
  }
  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::module_call_end_ack(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  end_req(ret);
  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::module_call_notify_req(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  error_notify(ret);
  return E_SUCCESS;
}

err_code_t UpgradeHostToModule::start_ack(sacp_hmi_message_t *msg, uint8_t ret) {
  return host_hmi.send_ack(msg, ret);
}

void UpgradeHostToModule::trans_data_req(uint32_t offset, uint16_t len) {
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
  LOG_I("%dms upgrade_hm: trans_req offset %d, buffer %d\r\n", millis(), offset, len);
  host_hmi.send(&tx_msg);
}

void UpgradeHostToModule::end_req(uint8_t ret) {
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
  LOG_I("%dms upgrade_hm: end_req, ret %d\r\n", millis(), ret);

  last_end_req_ms = millis();
  end_req_try++;
}

void UpgradeHostToModule::error_notify(uint8_t ret) {
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
  LOG_I("%dms upgrade_hm: notify %d\r\n", millis(), ret);
}
