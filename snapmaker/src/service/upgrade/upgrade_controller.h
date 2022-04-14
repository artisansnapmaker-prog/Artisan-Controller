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

#ifndef SNAPMAKER_UPGRADE_CONTROLLER_H_
#define SNAPMAKER_UPGRADE_CONTROLLER_H_

#include "../src/config.h"
#include "../../common/error.h"
#include "../../host/sacp_hmi.h"
#include "../../boot/boot.h"

// Forward declaration
class UpdateService;

class UpgradeCtrlService {
  public:
    UpgradeCtrlService(){};
    err_code_t init(UpdateService *s);
    err_code_t proc(boot_info_t *boot_info, sacp_hmi_message_t *msg);

  private:
    UpdateService *ugr_svc;
};

#endif  // #ifndef SNAPMAKER_CLIENT_NODE_H_
