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

#include "upgrade_module_interface.h"
#include "upgrade_controller_to_module.h"
#include "upgrade_host_to_module.h"
#include "upgrade_service.h"
#include "esp32_upgrade.h"
#include "sm2_upgrade.h"
#include "../../snapmaker.h"
#include "../../module/toolhead_laser.h"

/*
typedef struct {
  ugr_module_start_req start_req;
  ugr_module_start_ack start_ack;

  ugr_module_ready_req ready_req;
  ugr_module_ready_ack ready_ack;
  ugr_module_start_trans start_trans;
  
  ugr_module_trans_req trans_req;
  ugr_module_trans_ack trans_ack;
  
  ugr_module_end_req end_req;
  ugr_module_end_ack end_ack;
  
  ugr_module_notify_req notify_req;
  ugr_module_notify_ack notify_ack;
} UpgradeModuleHandle;
*/

UpgradeModuleInfo upgrade_module_info_tab[] = {
  
  {
    ESP32_FW,                                             /* packet type */
    0,                                                    /* start id    */
    0,                                                    /* end id      */
    esp32_camera_upgrade_handle_init,
    esp32_camera_upgrade_handle_deinit,
    {
      esp32_camera_upgrade_start, 
      module_call_start_ack,

      NULL, 
      NULL,
      NULL,
      
      module_call_trans_req, 
      esp32_camera_upgrade_trans, 
      
      esp32_camera_upgrade_end, 
      module_call_end_ack, 
      
      module_call_notify_req,
      NULL, 
    }
  },

  {
    SM2_MODULE_FW,                                        /* packet type */
    MODULE_DEVICE_ID_FDM_1EXTRUDER_2019,                  /* start id    */
    MODULE_DEVICE_ID_CALIBRATOR,                          /* end id      */
    sm2_module_upgrade_handle_init,
    sm2_module_upgrade_handle_deinit,
    {
      sm2_module_upgrade_start_req,
      module_call_start_ack,

      sm2_module_upgrade_ready_req,
      module_call_ready_ack,
      sm2_module_upgrade_start_trans,

      module_call_trans_req, 
      sm2_module_upgrade_trans_ack, 
      
      sm2_module_upgrade_end_req,
      module_call_end_ack,

      module_call_notify_req,
      NULL,
    }
  },
};

UpgradeModuleInfo *get_module_upgrade_handls(UpdatePackType pack_type, uint16_t id) {
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

err_code_t module_call_start_ack(uint8_t ret) {
  // FUN_LOG();
  if (UPGRADE_PHASE_HOST_TO_MODULE == upgrade_svc.get_upgrade_pahse()){
    return ugr_hm_svc.module_call_start_ack(ret);
  }
  else if (UPGRADE_PHASE_CONTROLLER_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_cm_svc.module_call_start_ack(ret);
  }
  else {
    return E_SUCCESS;
  }
}

err_code_t module_call_ready_ack(uint8_t ret) {
  // FUN_LOG();
  if (UPGRADE_PHASE_HOST_TO_MODULE == upgrade_svc.get_upgrade_pahse()){
    return E_SUCCESS;
  }
  else if (UPGRADE_PHASE_CONTROLLER_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_cm_svc.module_call_ready_ack(ret);
  }
  else {
    return E_SUCCESS;
  }
}

err_code_t module_call_trans_req(uint32_t req_offset, uint32_t len) {
  // FUN_LOG();
  if (UPGRADE_PHASE_HOST_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_hm_svc.module_call_trans_req(req_offset, len);
  }
  else if (UPGRADE_PHASE_CONTROLLER_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_cm_svc.module_call_trans_req(req_offset, len);
  }
  else {
    return E_SUCCESS;
  }
}

err_code_t module_call_end_ack(uint8_t ret) {
  // FUN_LOG();
  if (UPGRADE_PHASE_HOST_TO_MODULE == upgrade_svc.get_upgrade_pahse()){
    return ugr_hm_svc.module_call_end_ack(ret);
  }
  else if (UPGRADE_PHASE_CONTROLLER_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_cm_svc.module_call_end_ack(ret);
  }
  else {
    return E_SUCCESS;
  }
}

err_code_t module_call_notify_req(uint8_t ret) {
  // FUN_LOG();
  if (UPGRADE_PHASE_HOST_TO_MODULE == upgrade_svc.get_upgrade_pahse()){
    return ugr_hm_svc.module_call_notify_req(ret);
  }
  else if (UPGRADE_PHASE_CONTROLLER_TO_MODULE == upgrade_svc.get_upgrade_pahse()) {
    return ugr_cm_svc.module_call_notify_req(ret);
  }
  else {
    return E_SUCCESS;
  }
}

