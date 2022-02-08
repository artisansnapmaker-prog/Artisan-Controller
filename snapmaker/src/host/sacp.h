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
#ifndef SNAPMAKER_HOST_SACP_H_
#define SNAPMAKER_HOST_SACP_H_

#include "base.h"
#include "link_uart.h"
#include "link_can.h"

/*
  This host handle also SSTP protocol
*/


typedef struct {
  uint32_t peer;

  uint8_t  ver;
  uint8_t  attr;
  uint32_t seq;

  uint8_t cmd_set;
  uint8_t cmd_id;

  uint16_t length;
  uint8_t  *data;
} sacp_message_t;


// TODO: how to construct the event callbacks struct? there is only one byte for command id in some condition
class HostSACP: public HostBase {
  // public methods
  public:
    HostSACP(LinkBase &l): HostBase(), link(l) {}

    int init();

    int register_callback(uint8_t cmd_set, uint8_t cmd_id, std::function <int(sacp_message_t)> cb);
    int send_sync(sacp_message_t *in, sacp_message_t *out, uint32_t timeout=100, uint8_t retry=1);
    int send(sacp_message_t *in);

    // to be compatible with the condition only has one byte for command id
    int register_callback_legacy(uint8_t cmd_id, std::function <int(sacp_message_t)> cb);

    // these two API, only accept cmd_id in message
    int send_sync_legacy(sacp_message_t *in, sacp_message_t *out, uint32_t timeout=100, uint8_t retry=1);
    int send_legacy(sacp_message_t *in);

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    LinkBase &link;
};

// initalized in system thread
extern HostSACP host_hmi(link_hmi);
extern HostSACP host_luban(link_luban);

// initalized in laser init();
extern HostSACP host_camera(link_camera);

// initalized in module service init();
extern HostSACP host_can_cfg(link_can_cfg);

#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
