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

err_code_t UpgradeModuleService::init(UpdateService *s) {
  return E_SUCCESS;
}

err_code_t UpgradeModuleService::proc(boot_info_t *boot_info, sacp_hmi_message_t *msg) {
  return E_SUCCESS;
}

ModuleUpgradeType UpgradeModuleService::packet_upgrade_type(boot_info_t *boot_info) {

  if (ESP32_FW == boot_info->pack_type) {
    return UPGRADE_MODULE_BY_HOST;
  }
  else if (SM2_MODULE_FW == boot_info->pack_type) {
    if (boot_info->fw_lenght > module_fw_partition.size)
      return UPGRADE_MODULE_BY_HOST;
    else 
      return UPGRADE_MODULE_BY_CONTROLLER;
  }
  else {
    return UPGRADE_MODULE_BY_HOST;
  }

}
