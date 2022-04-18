/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2022 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Controller2022-Marlin)
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

#ifndef SNAPMAKER_ESP32_UPGRADE_H_
#define SNAPMAKER_ESP32_UPGRADE_H_

#include "../../common/error.h"
#include "../../host/sacp_hmi.h"
#include "upgrade_host_to_module.h"

#define ESP32_UPDATE_OPCODE_START_NOTIFY         (0x00)
#define ESP32_UPDATE_OPCODE_TRANS_NOTIFY         (0x01)
#define ESP32_UPDATE_OPCODE_END_NOTIFY           (0x02)
#define ESP32_UPDATE_OPCODE_FAIL_NOTIFY          (0xFF)

#define ESP32_FW_PACK_INDEX_INVALID             (0xFFFFFFFF) 
#define ESP32_FW_FILE_OFFSET_INVALID            (0xFFFFFFFF) 
#define ESP32_FW_PACK_MAX_LEN                   (UPGRADE_TRANS_BUF_SIZE)

// esp32 upgrade API
err_code_t esp32_camera_upgrade_handle_init(UpgradeModuleHandle *func_tab);
void esp32_camera_upgrade_handle_deinit(void);
err_code_t esp32_camera_upgrade_start(pack_info_t *);
err_code_t esp32_camera_upgrade_trans(uint32_t offset, uint8_t *data, uint32_t len);
err_code_t esp32_camera_upgrade_end(uint8_t end_type);
err_code_t esp32_camera_upgrade_start_ack_cb(void *obj, sacp_hmi_message_t *msg);
err_code_t esp32_camera_get_package_ack_cb(void *obj, sacp_hmi_message_t *msg);
err_code_t esp32_camera_updgrade_end_cb(void *obj, sacp_hmi_message_t *msg);
err_code_t esp32_camera_upgrade_fail_notify_cb(void *obj, sacp_hmi_message_t *msg);
#endif
