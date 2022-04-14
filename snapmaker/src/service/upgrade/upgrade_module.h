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

#ifndef SNAPMAKER_UPGRADE_MODULE_H_
#define SNAPMAKER_UPGRADE_MODULE_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../src/config.h"
#include "../../common/error.h"
#include "../../common/ring_buffer.h"
#include "../../common/type.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"


enum ModuleUpgradeType {
  UPGRADE_MODULE_BY_CONTROLLER = 0,
  UPGRADE_MODULE_BY_HOST,
};

class UpdateService;


/************************************************************************/
// module upgrade class
/************************************************************************/
class UpgradeModuleService {
  public:
    UpgradeModuleService(){};
    err_code_t init(UpdateService *s);
    err_code_t proc(boot_info_t *boot_info, sacp_hmi_message_t *msg);
    ModuleUpgradeType packet_upgrade_type(boot_info_t *boot_info);

  private:
    UpdateService *ugr_svc;
    ModuleUpgradeType ugr_type;
};


#endif  // #ifndef SNAPMAKER_UPGRADE_MODULE_H_
