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
#ifndef SNAPMAKER_HOST_LINK_CAN_H_
#define SNAPMAKER_HOST_LINK_CAN_H_

#include "link.h"

enum LinkCANType {
  LINK_CAN_EXT_REMOTE,  // scan channel
  LINK_CAN_EXT_DATA,    // configuration channel
  LINK_CAN_STD_DATA,    // routine channel
  LINK_CAN_INVALID
};

class LinkCAN: public LinkBase {
  public:
    LinkCAN() {}
    LinkCAN(LinkCANType type): LinkBase() {}
};

extern LinkCAN link_can_scan;
extern LinkCAN link_can_cfg;
extern LinkCAN link_can_rou;

#endif  // #ifndef SNAPMAKER_HOST_LINK_CAN_H_
