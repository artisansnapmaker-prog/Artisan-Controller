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
#ifndef SNAPMAKER_HOST_SM_CAN_H_
#define SNAPMAKER_HOST_SM_CAN_H_

#include "base.h"
#include "link_can.h"

#include "../module/base.h"

typedef void (*msg_handle)(void *obj, uint8_t *, uint8_t);

typedef struct {
  void *obj;
  msg_handle callback;
} msg_callback_t;

class HostSMCAN: public HostBase {
  // public methods
  public:
    HostSMCAN(LinkCANStdData &l): HostBase(), link(l) {}

    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);

    err_code_t send(LinkCANChannel ch, uint16_t msg_id, uint8_t *data, uint8_t length);

    err_code_t register_callback(uint16_t msg_id, void *obj, msg_handle cb);

    int handle_receive();
    int handle_events();

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    LinkCANStdData &link;
    MessageBufferHandle_t recv_queue;

    msg_callback_t callbacks[MODULE_SUPPORT_MESSAGE_ID_MAX];
};

extern HostSMCAN host_can_rou(link_can_rou);

#endif  // #ifndef SNAPMAKER_HOST_SM_CAN_H_
