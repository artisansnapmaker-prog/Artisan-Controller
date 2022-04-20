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
#include "upgrade_controller_to_module.h"
#include "upgrade_service.h"
#include "../../snapmaker.h"
#include "../module.h"

UpgradeControllerToModule ugr_cm_svc;

err_code_t UpgradeControllerToModule::init(UpdateService *s) {
  ugr_svc = s;
  module_index = MODULE_ACCESSIBLE_MAX;
  status = UPGRADE_CM_STATUS_IDLE;
  return E_SUCCESS;
}

void UpgradeControllerToModule::loop(void) {

  switch(status) {
    case UPGRADE_CM_STATUS_IDLE:
    break;

    case UPGRADE_CM_STATUS_A_MODULE_START:
      module = get_next_module();
      if (module) {
        start_a_module_upgrade();
      }
      else {
        reset_to_idle();
      }
    break;

    case UPGRADE_CM_STATUS_START:
      if (time_after(millis(), last_action_ms + 500)) {
        if (action_req_try < 10) {
          action_req_try++;
          last_action_ms = millis();
        }
        else {
          LOG_E("upgrade_cm: upgrade start request timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_CM_STATUS_WAIT_FOR_READY:
      if (time_after(millis(), last_action_ms + 500)) {
        if (action_req_try < 20) {
          action_req_try++;
          last_action_ms = millis();
          if (0 == action_req_try%5)
            ready_req();
        }
        else {
          LOG_E("upgrade_cm: upgrade start request timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_CM_STATUS_TRANS:
      if (time_after(millis(), last_action_ms + 500)) {
        if (action_req_try < 10) {
          action_req_try++;
          last_action_ms = millis();
        }
        else {
          LOG_E("upgrade_cm: upgrade trans timeout, return to upgrade init\r\n");
          reset_to_idle();
        }
      }
    break;

    case UPGRADE_CM_STATUS_END:
      LOG_I("upgrade_cm: finish a module, return to UPGRADE_CM_STATUS_A_MODULE_START and restart another moduel upgrade\r\n");
      status = UPGRADE_CM_STATUS_A_MODULE_START;
    break;

    default:
    break;
  }

}

void UpgradeControllerToModule::reset_to_idle(void) {
  if (module_upgrade_info) {
    module_upgrade_info->module_deinit();
  }
  status = UPGRADE_CM_STATUS_IDLE;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_INIT);
  smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL);
}

err_code_t UpgradeControllerToModule::start(void) {

  pit = (pack_info_t *)module_fw_partition.start_addr;
  if (!boot_info_check(pit)) {
    LOG_E("upgrade_cm: packet info checksum failure\r\n");
    return E_FAILURE;
  }
  
  module_index = 0;
  module = get_next_module();
  if (module) {
    return start_a_module_upgrade();
  }
  else {
    return E_FAILURE;
  }
}

ModuleBase *UpgradeControllerToModule::get_next_module(void) {
  ModuleBase *m;

  while(module_index < MODULE_ACCESSIBLE_MAX) {
    m = module_svc.get_module(module_index);
    module_index += 1;
    if (!m)
      continue;

    uint16_t id = m->get_device_id();
    if (pit->start_index <= id && id <= pit->end_index) {
      return m;
    }
    else {
      continue;
    }
  }

  return NULL;
}

err_code_t UpgradeControllerToModule::module_call_start_ack(uint8_t ret) {
  FUN_LOG();
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  if (ret == E_SUCCESS) {
    LOG_I("upgrade_cm: change to UPGRADE_CM_STATUS_WAIT_FOR_READY\r\n");
    status = UPGRADE_CM_STATUS_WAIT_FOR_READY;
    action_req_try = 0;
    last_action_ms = millis();
    ready_req();
  }
  else {
    reset_to_idle();
  }

  return E_SUCCESS;
}

err_code_t UpgradeControllerToModule::module_call_ready_ack(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  if (ret == E_SUCCESS) {
    LOG_I("upgrade_cm: start trans\r\n");
    action_req_try = 0;
    last_action_ms = millis();
    start_trans();
    status = UPGRADE_CM_STATUS_TRANS;
  }
  else {
    reset_to_idle();
  }

  return E_SUCCESS;
}

err_code_t UpgradeControllerToModule::module_call_trans_req(uint32_t req_offset, uint32_t len) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  LOG_I("upgrade_cm: req offset %d\r\n", req_offset);
  offset = req_offset;
  trans_len = len;
  configASSERT(module_upgrade_info);
  if(E_SUCCESS == module_upgrade_info->handle.trans_ack(offset, 
                                                        (uint8_t *)(fw_flash_addr + offset),
                                                        UPGRADE_CM_TRANS_BUF_SIZE)) {
    offset += UPGRADE_CM_TRANS_BUF_SIZE;
  }
  action_req_try = 0;
  last_action_ms = millis();

  if (offset >= fw_lenght) {
    LOG_I("upgrade_cm: TX all the data\r\n");
    configASSERT(module_upgrade_info);
    end_ret = E_SUCCESS;
    module_upgrade_info->handle.end_req(end_ret);
    status = UPGRADE_CM_STATUS_END;
  }

  return E_SUCCESS;
}

err_code_t UpgradeControllerToModule::module_call_end_ack(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t UpgradeControllerToModule::module_call_notify_req(uint8_t ret) {
  if (!module_upgrade_info) {
    return E_FAILURE;
  }

  return E_SUCCESS;
}

err_code_t UpgradeControllerToModule::start_a_module_upgrade(void) {

  module_upgrade_info = get_module_upgrade_handls(  (UpdatePackType)pit->pack_type, 
                                                    module->get_device_id());
  if (!module_upgrade_info) {
    LOG_E("upgrade_cm: unsupported pack %d with id %d\r\n", SM2_MODULE_FW, module->get_device_id());
    return E_FAILURE;
  }

  if (E_SUCCESS != module_upgrade_info->module_init(&(module_upgrade_info->handle))) {
    LOG_E("upgrade_cm: module upgrade init error\r\n");
    return E_FAILURE;
  }

  if (E_SUCCESS != smprinter.set_sys_status(SYSTEM_STATUS_MODULE_UPGRADE, NULL)) {
    LOG_E("upgrade_cm: can not enter module upgrade status\r\n");
    return E_FAILURE;
  }

  fw_flash_addr = module_fw_partition.start_addr + BOOT_INFO_SIZE;
  fw_lenght = pit->fw_lenght;
  fw_checksum = pit->fw_checksum;
  fw_id = module->get_device_id();
  offset = 0;
  module_info.mac = module->get_mac();
  module_info.ch = module->get_channel();

  LOG_I("upgrade_cm: module %d:%d start upgraed\r\n", module->get_device_id(), module->get_mac());
  status = UPGRADE_CM_STATUS_START;
  ugr_svc->set_updgrade_phase(UPGRADE_PHASE_CONTROLLER_TO_MODULE);

  last_action_ms = millis();
  action_req_try = 0;
  module_upgrade_info->handle.start_req(pit, &module_info);

  return E_SUCCESS;
}

void UpgradeControllerToModule::ready_req(void) {
  configASSERT(module_upgrade_info);
  module_upgrade_info->handle.ready_req();
}

void UpgradeControllerToModule::start_trans(void) {
  configASSERT(module_upgrade_info);
  module_upgrade_info->handle.start_trans();
  last_action_ms = millis();
  action_req_try++;
}

void UpgradeControllerToModule::trans_data_req(uint32_t offset, uint16_t len) {
  LOG_I("%dms upgrade_module: trans_req offset %d, buffer %d\r\n", millis(), offset, len);

  configASSERT(module_upgrade_info);
  if(E_SUCCESS == module_upgrade_info->handle.trans_ack(offset, 
                                                        (uint8_t *)(fw_flash_addr + offset),
                                                        UPGRADE_CM_TRANS_BUF_SIZE)) {
    offset += UPGRADE_CM_TRANS_BUF_SIZE;
  }

  last_action_ms = millis();
  action_req_try++;
}

