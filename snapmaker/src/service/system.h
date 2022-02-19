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
#ifndef SNAPMAKER_SYSTEM_SERVICE_H_
#define SNAPMAKER_SYSTEM_SERVICE_H_

#include <stdint.h>

enum MachineModel {
  MACHINE_MODEL_A150,
  MACHINE_MODEL_A250,
  MACHINE_MODEL_A350,
  MACHINE_MODEL_A400,
  MACHINE_MODEL_J1
};


enum SystemStatus {
  SYSTEM_STATUS_INIT,
  SYSTEM_STATUS_IDLE,

  SYSTEM_STATUS_WORKING,

  SYSTEM_STATUS_UPGRADING,

  SYSTEM_STATUS_PAUSE_TRIG,
  SYSTEM_STATUS_PAUSE_PARK,
  SYSTEM_STATUS_PAUSE_FINISH,

  SYSTEM_STATUS_RESUME_TRIG,
  SYSTEM_STATUS_RESUME_MOVING,
  SYSTEM_STATUS_RESUME_WAITING,

  SYSTEM_STATUS_STOP_TRIG,
  SYSTEM_STATUS_STOP_FINISH,

  SYSTEM_STATUS_INVALID
};


class SystemService {
  // public methods
  public:
    SystemService() {}

    void init() {}
    void background_thread() { return ; }

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    MachineModel model;         // machine model
    uint8_t      hw_ver;        // controller hardware version
    uint32_t     sn;            // controller serial number
    char         fw_ver[33];    // controller version
};


extern SystemService system_svc;


#endif
