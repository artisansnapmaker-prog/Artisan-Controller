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

#ifndef SNAPMAKER_SM2_UPGRADE_H_
#define SNAPMAKER_SM2_UPGRADE_H_

#include "../../common/error.h"
#include "../../host/sacp_module.h"
#include "upgrade_module_interface.h"


// sm2 module upgrade API
err_code_t sm2_module_upgrade_init(void);

err_code_t sm2_module_upgrade_handle_init(UpgradeModuleHandle *func_tab);
void sm2_module_upgrade_handle_deinit(void);

err_code_t sm2_module_upgrade_start_req(pack_info_t *, module_info_t *m);
err_code_t sm2_module_upgrade_start_ack_cb(void *obj, sacp_module_message_t *msg);

err_code_t sm2_module_upgrade_ready_req(void);
err_code_t sm2_module_upgrade_ready_ack(void *obj, sacp_module_message_t *msg);
err_code_t sm2_module_upgrade_start_trans(void);

err_code_t sm2_module_upgrade_trans_req(void *obj, sacp_module_message_t *msg);
err_code_t sm2_module_upgrade_trans_ack(uint32_t offset, uint8_t *data, uint32_t len);

err_code_t sm2_module_upgrade_end_req(uint8_t end_type);


#endif
