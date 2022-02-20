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
#ifndef SNAPMAKER_HOST_LINK_UART_H_
#define SNAPMAKER_HOST_LINK_UART_H_

class LinkUART {
  // public methods
  public:
    LinkUART() {}



  // private methods
  private:


  // public properties
  public:


  // private properties
  private:

};

extern LinkUART link_hmi;
extern LinkUART link_camera;
extern LinkUART link_luban;

#endif  // #ifndef SNAPMAKER_HOST_LINK_UART_H_
