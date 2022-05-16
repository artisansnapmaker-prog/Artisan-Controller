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
#include "../link/link_can.h"
#include "../link/link_uart.h"

/*
  This host handle also SSTP protocol, but its verion = 0
*/

#define SACP_PDU_MAX_SIZE   (1024)
#define SACP_FRAME_SOF_1  (0xaa)
#define SACP_FRAME_SOF_2  (0x55)

#define SACP_FRONT_HEADER_MIN_SIZE  (7)
#define SACP_FRAME_INDEX_VER        (4)

#define SACP_CMD_ID_INVALID   (0xFFFF)

#define SACP_HOST_INVALID     (0xFFFFFFFF)

// for attributions
// bit[7:0] is on the packet defination
#define SACP_MESSAGE_ATTR_ACK        (0x00000001)

// bit[31:8] is used to tell API some info
#define SACP_MESSAGE_ATTR_SET_SEQ    (0x80000000) // indicates we want to use customize sequence to send message
#define SACP_MESSAGE_ATTR_SET_VER    (0x40000000) // indicates we want to use customize sequence to send message

// #defination for callback attribution
#define SACP_CB_ATTR_ACK                     (0x00000001)
#define SACP_CB_ATTR_BLOCKED_WITH_MOTION     (0x00000002)
#define SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION  (0x00000004)

enum SACPCommandSet {
  SACP_CMD_SET_GLOBAL_REQ                 = 0x1,

  SACP_CMD_SET_NOTIFICATION               = 0x4,

  SACP_CMD_SET_FDM                        = 0x10,
  SACP_CMD_SET_CNC                        = 0x11,
  SACP_CMD_SET_LASER                      = 0x12,
  SACP_CMD_SET_LINEAR_MODULE              = 0x13,
  SACP_CMD_SET_HEATED_BED                 = 0x14,
  SACP_CMD_SET_ENCLOSURE                  = 0x15,
  SACP_CMD_SET_ROTARY_MODULE              = 0x16,
  SACP_CMD_SET_AIR_PURIFIER               = 0x17,
  SACP_CMD_SET_DRY_BOX                    = 0x18,

  SACP_CMD_SET_CALIBRATE_FDM              = 0xa0,
  SACP_CMD_SET_CALIBRATE_CNC              = 0xa4,
  SACP_CMD_SET_CALIBRATE_LASER            = 0xa8,
  SACP_CMD_SET_CAMERA                     = 0xa9,
  SACP_CMD_SET_WOKRING_FLOW               = 0xac,
  SACP_CMD_SET_UPGRADE                    = 0xad,

  SACP_CMD_SET_MAX
};

#define SACP_CMD_ID_GLOABL_REQ_SUBSCRIPT      (0x00)
#define SACP_CMD_ID_GLOABL_REQ_UNSUBSCRIPT    (0x01)
#define SACP_CMD_ID_GLOABL_REQ_REBOOT         (0x03)

#define SSTP_ESP32_UPDATE_FW_EVENT_ASK        (0x10)

#define SACP_CMD_ID_GLOABL_REQ_SET_PC_PROTOCOL  (0x11)
#define SACP_CMD_ID_GLOABL_REQ_FACTORY_RESET    (0x12)

#define SACP_CMD_ID_GLOABL_REQ_GET_MODULE_INFO  (0x20)
#define SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO (0x21)
#define SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE (0x22)

#define SACP_CMD_ID_GLOABL_REQ_GET_COORDINATE         (0x30)
#define SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE  (0x31)
#define SACP_CMD_ID_GLOABL_REQ_SET_ORIGIN             (0x32)
#define SACP_CMD_ID_GLOABL_REQ_MOVE_ABSOLUTELY        (0x34)
#define SACP_CMD_ID_GLOABL_REQ_HOME                   (0x35)
#define SACP_CMD_ID_GLOABL_REQ_REPORT_HOME_RESULT     (0x36)

#define SACP_CMD_ID_GLOABL_REQ_NOTIFY_EMERGENCY_STOP  (0x3b)

#define SACP_CMD_ID_GLOABL_REQ_ENTRY_REPLACE_MODE     (0x3d)

#define SACP_CMD_ID_GLOABL_REQ_HEARTBEAT          (0xa0)
#define SACP_CMD_ID_GLOABL_REQ_SUB_COORDINATE     (0xa2)

enum SACPCommandIdNotification {
  SACP_CMD_ID_NOTIFICATION_RAISE_EXCEPTION = 0,
  SACP_CMD_ID_NOTIFICATION_CEALR_EXCEPTION,
  SACP_CMD_ID_NOTIFICATION_GET_EXCEPTION,
};


typedef struct {
  uint32_t peer;

  uint8_t  ch;
  uint8_t  ver;
  uint32_t  attr;
  uint32_t seq;

  uint16_t cmd_set;
  uint16_t cmd_id;

  uint16_t length;
  uint8_t  *data;
} sacp_message_t;

enum SACPHostID {
  SACP_HOST_ID_LUBAN,
  SACP_HOST_ID_CONTROLLER,
  SACP_HOST_ID_SCREEN,
  SACP_HOST_ID_ESP32,

  SACP_HOST_ID_INVALIDE = 0xffffffff,
};

enum SACPVerion {
  SACP_VER_0,
  SACP_VER_1,

  SACP_VER_INVALID
};

enum SACPParserStatus {
  SACP_PARSER_STA_IDLE,
  SACP_PARSER_STA_GOT_SOF,
  SACP_PARSER_STA_GOT_HEAD,
  SACP_PARSER_STA_GOT_LENGTH,
  SACP_PARSER_STA_GOT_MESSAGE,

  SACP_PARSER_STA_INVALID
};
typedef struct {
  uint8_t           ver;
  SACPParserStatus status;
  uint32_t         next_timeout;
  uint16_t         length;
  uint8_t buffer[SACP_PDU_MAX_SIZE];
} sacp_parser_t;


typedef struct {
  uint32_t      seq;
  LinkUART      *link;
  sacp_parser_t parser;
  SemaphoreHandle_t lock;
} sacp_channel_t;

// V0 definations ++++++++++++++

#define SACP_V0_MODULE_MIN_SIZE   (9)
#define SACP_V0_HMI_MIN_SIZE      (10)
#define SACP_V0_HEADER_SIZE       (6)
#define SACP_V0_REAR_HEADER_SIZE  (4)
#define SACP_V0_NON_PAYPLOAD_SIZE (8)

#define SACP_V0_PDU_MIN_SIZE  (9)

// use big ending when V0
enum SACPV0FrameIndex {
  SACP_V0_FRAME_INDEX_SOF_1,
  SACP_V0_FRAME_INDEX_SOF_2,
  SACP_V0_FRAME_INDEX_LEN_H,
  SACP_V0_FRAME_INDEX_LEN_L,
  SACP_V0_FRAME_INDEX_VER,
  SACP_V0_FRAME_INDEX_LEN_CHK,
  SACP_V0_FRAME_INDEX_CHK_H,
  SACP_V0_FRAME_INDEX_CHK_L,
  SACP_V0_FRAME_INDEX_EVENT_ID,
  SACP_V0_FRAME_INDEX_OPCODE,
  SACP_V0_FRAME_INDEX_DATA,
};

typedef struct {
  uint32_t peer;
  uint8_t  ch;
  uint8_t  cmd_id;
  uint16_t length;
  uint8_t  *data;
} sacp_module_message_t;



// V1 definations ++++++++++++++

#define SACP_V1_PDU_MAX_SIZE    (SACP_PDU_MAX_SIZE)
#define SACP_V1_PDU_MIN_SIZE      (15)
#define SACP_V1_FRONT_HEADER_SIZE (7)
#define SACP_V1_REAR_HEADER_SIZE  (6)

#define SACP_V1_HOST_INVALID    (SACP_HOST_INVALID)
#define SACP_V1_SEQ_INVALID     (0xFFFFFFFF)
#define SACP_V1_CMD_SET_INVALID (0xFFFF)
#define SACP_V1_CMD_ID_INVALID  (0xFFFF)


#define SACP_PDU_MIN_SIZE   (SACP_V0_PDU_MIN_SIZE)

typedef sacp_message_t sacp_hmi_message_t;

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
  SACP_V1_FRAME_INDEX_PAYLOAD
};

enum SACPWaitingNodeStatus {
  SACP_WAITING_NODE_STA_IDLE,
  SACP_WAITING_NODE_STA_INUSE_V0,
  SACP_WAITING_NODE_STA_INUSE_V0_LEGACY,
  SACP_WAITING_NODE_STA_INUSE_V1,

  SACP_WAITING_NODE_STA_INVALID
};

enum SACPRouteStatus {
  SACP_ROUTE_STA_OFFLINE,
  SACP_ROUTE_STA_ONLINE,

  SACP_ROUTE_STA_INVALID
};

typedef struct {
  uint32_t peer;
  uint8_t  ch;
  uint8_t  ver;
  SACPRouteStatus status;  // online or offline
} sacp_route_table_t;

class HostSACP: public HostBase {
  // public methods
  public:
    HostSACP(): HostBase() {}

    uint16_t calculate_checksum(uint8_t *buffer, uint16_t length);

  // private methods
  protected:
    err_code_t package(sacp_module_message_t *message, uint8_t *pdu, uint16_t *pdu_len);
    err_code_t package(sacp_message_t *message, uint8_t *pdu, uint16_t *pdu_len);

    uint8_t calc_crc8(uint8_t *data, uint16_t length);

  private:
    uint16_t package_v0(uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t event_id=0xFFFF, uint16_t opcode=0xFFFF);
    uint16_t package_v1(sacp_message_t *msg, uint8_t *out);


  // public properties
  public:


  // protected properties
  protected:
    SACPVerion version;

    // host id
    uint32_t host_id;

};

#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
