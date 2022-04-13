/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker step servervo
 * (see https://github.com/Snapmaker/stepservo-xdrive)
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

/*****
  ** @file     : sacp_protocol.c/h
  ** @brief    : none
  ** @versions : 1.0.0
  ** @time     : 2021/12/01
  ** @reviser  : 747
  ** @explain  : null
*****/

#ifndef SNAPMAKER_BOOT_PROTOCOL_H
#define SNAPMAKER_BOOT_PROTOCOL_H


/********************************************************************************/
// INCLUDE
/********************************************************************************/
#include <stdint.h>


/********************************************************************************/
// MICRO DEF
/********************************************************************************/
#define SACP_FRAME_MAX_SIZE           (2048)
#define HEADER_1                      (0xAA)  
#define HEADER_2                      (0x55)  
#define CMD_CODE_POS                  (0)
#define SEQ_POS                       (1)
#define HEADER_LEN                    (7)
#define PAYLOAD_ADDITION_LEN          (6)
#define FRAME_MIN_LEN                 (13)
#define PAYLOAD_POS                   (11)
#define VERSION                       (0x01)
#define SACP_PAYLOAD_MAX_SIZE         (SACP_FRAME_MAX_SIZE - FRAME_MIN_LEN)

#define ATTR_REQ                      (0x00)
#define ATTR_ACK                      (0x01)


/********************************************************************************/
// TYPEDEF
/********************************************************************************/
enum SACPHostID {
  SACP_HOST_ID_LUBAN,
  SACP_HOST_ID_CONTROLLER,
  SACP_HOST_ID_SCREEN,
};

typedef enum{
  STATE_HEADER_1 = 0,
  STATE_HEADER_2,
  STATE_LEN1,
  STATE_LEN2,
  STATE_VER,
  STATE_RECEIVER,
  STATE_CRC,
  STATE_SENDER,
  STATE_ATTR,
  STATE_SEQ1,
  STATE_SEQ2,
  STATE_DATA,
  STATE_CHECKSUM1,
  STATE_CHECKSUM2,
} fsm_state_e;

typedef struct{
  fsm_state_e s;
  uint32_t last_milli;
  uint16_t len;
  uint8_t ver;
  uint8_t receiver;
  uint8_t header_crc8;
  uint8_t sender;
  uint8_t attr;
  uint16_t seq;
  uint16_t checksum;
  uint16_t have_rx_len;
  uint8_t frame[SACP_FRAME_MAX_SIZE];
  uint8_t *payload;
  uint32_t payload_len;
} fsm_info_t;

typedef struct{
  uint8_t ver;
  uint8_t peer;
  uint8_t sender;
  uint8_t attr;
  uint8_t seq;
  uint32_t payload_len;
  uint8_t payload[SACP_FRAME_MAX_SIZE];
} scap_msg_t;


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/


/********************************************************************************/
// LOCAL FUN DECL
/********************************************************************************/


/********************************************************************************/
// EXP FUN DEF
/********************************************************************************/
void protocol_timeout_check(fsm_info_t &fsm);
int protocol_push_char(fsm_info_t &fsm, uint8_t c);
void print_frame(uint8_t *frame, uint32_t flen);
void protocol_build_pack(scap_msg_t &msg, uint8_t *frame_buf, uint32_t &out_frame_len);

#endif
