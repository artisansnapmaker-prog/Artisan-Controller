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

#include "../config.h"
#include "../common/error.h"

#include "base.h"

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
    HostSACP(): HostBase() {}


  // private methods
  private:


  // public properties
  public:


  // private properties
  private:

};

#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
