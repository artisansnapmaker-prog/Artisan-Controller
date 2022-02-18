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
  This host handle also SSTP protocol, but its verion = 0
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

enum SACPVerion {
  SACP_VER_0,
  SACP_VER_1,

  SACP_VER_INVALID
};

#define SACP_FRAME_SOF_1  (0xaa)
#define SACP_FRAME_SOF_2  (0x55)

// use big ending when V0
enum SACPV0FrameIndex {
  SACP_V0_FRAME_INDEX_SOF_1,
  SACP_V0_FRAME_INDEX_SOF_2,
  SACP_V0_FRAME_INDEX_LEN_H,
  SACP_V0_FRAME_INDEX_LEN_L,
  SACP_V0_FRAME_INDEX_VER,
  SACP_V0_FRAME_INDEX_LEN_CHK,
  SACP_V0_FRAME_INDEX_EVENT_ID,
  SACP_V0_FRAME_INDEX_OPCODE,
  SACP_V0_FRAME_INDEX_DATA,
};


// use little ending when V1
enum SACPV1FrameIndex {
  SACP_V1_FRAME_INDEX_SOF_1,
  SACP_V1_FRAME_INDEX_SOF_2,
  SACP_V1_FRAME_INDEX_LEN_L,
  SACP_V1_FRAME_INDEX_LEN_H,
  SACP_V1_FRAME_INDEX_VER,
  SACP_V1_FRAME_INDEX_RECV_ID,
  SACP_V1_FRAME_INDEX_CRC8,
  SACP_V1_FRAME_INDEX_SENDER_ID,
  SACP_V1_FRAME_INDEX_ATTR,
  SACP_V1_FRAME_INDEX_SEQ_L,
  SACP_V1_FRAME_INDEX_SEQ_H,
  SACP_V1_FRAME_INDEX_CMD_SET,
  SACP_V1_FRAME_INDEX_CMD_ID,
};

enum SACPWaitingNodeStatus {
  SACP_WAITING_NODE_STA_IDLE,
  SACP_WAITING_NODE_STA_INUSE,

  SACP_WAITING_NODE_STA_INVALID
};

class HostSACP: public HostBase {
  // public methods
  public:
    HostSACP(): HostBase() {}


  // private methods
  private:


  // public properties
  public:


  // protected properties
  protected:
    SACPVerion version;
};

#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
