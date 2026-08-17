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
#ifndef SNAPMAKER_HOST_SM_MAC_H_
#define SNAPMAKER_HOST_SM_MAC_H_

#include "base.h"
#include "../link/link_can.h"

typedef err_code_t (*smmac_callback)(void *obj, uint32_t mac, LinkCANChannel ch);

#define SM_MAC_RECV_BUFFER_SIZE (32)

class HostSMMAC: public HostBase {
  // public methods
  public:
    HostSMMAC(LinkCANExtRemote &l): HostBase(), link(l) {}

    err_code_t init(TaskHandle_t ev_task, SemaphoreHandle_t recv_event);

    err_code_t register_callback(void *obj, smmac_callback cb) {
      callback = cb;
      callback_obj = obj;
      return E_SUCCESS;
    }

    err_code_t send(uint32_t message);

    void handle_receive();

    void handle_events();

  // private properties
  private:
    RingBuffer<uint32_t>  recv_buffer;
    LinkCANExtRemote      &link;

    QueueHandle_t         event_queue;
    smmac_callback        callback = NULL;
    void                  *callback_obj;
};

extern HostSMMAC host_mac;

#endif  // #ifndef SNAPMAKER_HOST_SM_MAC_H_
