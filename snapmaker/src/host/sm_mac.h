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
#include "link_can.h"

class HostSMMAC: public HostBase {
  // public methods
  public:
    HostSMMAC(LinkBase &l): HostBase(), link(l) {}

    int init();

    int register_callback(std::function <int(uint32_t, uint8_t)> cb) { callback = cb; }

    int send(uint32_t message);

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    LinkBase &link;
    std::function <int(uint32_t, uint8_t)> callback;

};

extern HostSMMAC host_mac(link_can_scan);

#endif  // #ifndef SNAPMAKER_HOST_SM_MAC_H_
