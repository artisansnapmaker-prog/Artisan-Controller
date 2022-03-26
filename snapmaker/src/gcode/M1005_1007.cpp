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
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"

#include "../service/motion.h"
#include "../service/module.h"

#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

// marlin headers
#include "src/gcode/gcode.h"
#include "src/module/motion.h"


void GcodeSuite::M1005() {
  // MAC_t       mac;

  // char buffer[VERSION_STRING_SIZE + 4];
  // int  i;

  // version in code
  LOG_I("\nMarlin %s\n", SHORT_BUILD_VERSION);
  LOG_I("Compiled: %s, %s\n", __DATE__, __TIME__);

  // version in package
  // memcpy(buffer, (char*)(FLASH_BOOT_PARA + 2048), 30);
  // SERIAL_ECHO_MSG(MSG_MARLIN_PACK, ": ", buffer, "\n");


  // version of modules
  LOG_I("\nModule info:\n");
  ModuleBase *module;
  for (int i = 0; i < MODULE_ACCESSIBLE_MAX; i++) {
    module = module_svc.get_module(i);
    if (!module)
      break;

    LOG_I("Module: 0x%x, SN: 0x%0x, FW ver: %s, HW ver: 0x%x\n", module->get_device_id(),
          module->get_sn(), module->get_fw_version(), module->get_hw_verion());
  }

  // if (ModuleBase::toolhead() == MACHINE_TYPE_LASER || (ModuleBase::toolhead() == MACHINE_TYPE_LASER_10W)) {
  //   laser->ReadBluetoothVer();
  // }

  LOG_I("\nMachine Size: ");
  switch (smprinter.get_model()) {
  case SNAPMAKER_MODEL_A150:
    SERIAL_ECHOLN("S");
    break;

  case SNAPMAKER_MODEL_A250:
    SERIAL_ECHOLN("M");
    break;

  case SNAPMAKER_MODEL_A350:
    SERIAL_ECHOLN("L");
    break;

  case SNAPMAKER_MODEL_A400:
    SERIAL_ECHOLN("A400");
    break;

  case SNAPMAKER_MODEL_J1:
    SERIAL_ECHOLN("J1");
    break;

  default:
    SERIAL_ECHOLN("U");
    break;
  }
}


void GcodeSuite::M1006() {
  LOG_I("Tool Head: ");
  switch (smprinter.get_toolhead_type()) {
  case TH_TYPE_3DP:
    LOG_I("3DP\n");
    break;

  case TH_TYPE_LASER:
    LOG_I("LASER\n");
    LOG_I("Current Status: \n");
    // SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    // SERIAL_ECHO_MSG("Current Power: ", laser->power());
    // SERIAL_ECHO_MSG("Focus Height: ", laser->focus());
    break;

  case TH_TYPE_CNC:
    LOG_I("CNC\n");
    // SERIAL_ECHO_MSG("Current Power: ", cnc.power());
    // SERIAL_ECHO_MSG("RPM: ", cnc.rpm());
    break;

  default:
    LOG_I("UNKNOWN\n");
    break;
  }
}


void GcodeSuite::M1007() {
  int8_t active_coor = motion_svc.get_active_coordinate_system();
  xyz_pos_t pos_shift = motion_svc.get_position_shift();
  xyz_pos_t acvtive_coordinate = motion_svc.get_active_coordinate_system(active_coor);


  LOG_I("Homed: %s\n", motion_svc.is_all_axes_homed()? "YES" : "NO");

  LOG_I("Selected origin num: %d\n", active_coor + 1);

  LOG_I("Selected == Current: ");

  if (active_coordinate_system < 0) {
    LOG_I("YES\n");
  }
  else if ((pos_shift[X_AXIS] == acvtive_coordinate[X_AXIS]) &&
        (pos_shift[Y_AXIS] == acvtive_coordinate[Y_AXIS]) &&
        (pos_shift[Z_AXIS] == acvtive_coordinate[Z_AXIS]) &&
        (pos_shift[A_AXIS] == acvtive_coordinate[A_AXIS]) &&
        (pos_shift[B_AXIS] == acvtive_coordinate[B_AXIS])) {
    LOG_I("YES\n");
  }
  else {
    LOG_I("NO\n");
  }

  if (active_coordinate_system < 0) {
    LOG_I("Origin offset X: %.3f\n", pos_shift[X_AXIS]);
    LOG_I("Origin offset Y: %.3f\n", pos_shift[Y_AXIS]);
    LOG_I("Origin offset Z: %.3f\n", pos_shift[Z_AXIS]);
    LOG_I("Origin offset A: %.3f\n", pos_shift[A_AXIS]);
    LOG_I("Origin offset B: %.3f\n", pos_shift[B_AXIS]);
  }
  else {
    LOG_I("Origin offset X: %.3f\n", acvtive_coordinate[X_AXIS]);
    LOG_I("Origin offset Y: %.3f\n", acvtive_coordinate[Y_AXIS]);
    LOG_I("Origin offset Z: %.3f\n", acvtive_coordinate[Z_AXIS]);
    LOG_I("Origin offset A: %.3f\n", acvtive_coordinate[A_AXIS]);
    LOG_I("Origin offset B: %.3f\n", acvtive_coordinate[B_AXIS]);
  }
}
