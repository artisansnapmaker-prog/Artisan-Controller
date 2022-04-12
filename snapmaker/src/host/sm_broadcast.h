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
#ifndef SNAPMAKER_HOST_SM_BROADCASE_H_
#define SNAPMAKER_HOST_SM_BROADCASE_H_

#include "base.h"
#include "../link/link_can.h"

class HostSMBroadcast: public HostBase {
  // public methods
  public:
    HostSMBroadcast(LinkCANStdRemote &l): HostBase(), link(l) {}

    err_code_t init(TaskHandle_t ev_task, SemaphoreHandle_t recv_signal);
    err_code_t send(uint32_t message);

    void handle_receive() { return; }
    void handle_events() { return; }

  // private properties
  private:
    LinkCANStdRemote      &link;
};

extern HostSMBroadcast host_broadcast;

#endif
