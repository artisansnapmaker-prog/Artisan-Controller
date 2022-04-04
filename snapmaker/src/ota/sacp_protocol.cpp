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
  ** @file     : sacp_protocol.cpp/h
  ** @brief    : none
  ** @versions : 1.0.0
  ** @time     : 2021/12/01
  ** @reviser  : 747
  ** @explain  : null
*****/


/********************************************************************************/
// INCLUDE
/********************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "sacp_protocol.h"


/********************************************************************************/
// MICRO DEF
/********************************************************************************/


/********************************************************************************/
// TYPEDEF
/********************************************************************************/


/********************************************************************************/
// LOCAL FUN DECL
/********************************************************************************/
uint8_t calc_crc8(uint8_t *data, uint16_t length);
uint16_t calculate_checksum(uint8_t *buffer, uint16_t length);


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/


/********************************************************************************/
// EXP FUN DEF
/********************************************************************************/
bool protocol_push_char(fsm_info_t &fsm, uint8_t c)
{
  bool ret = false;

  switch(fsm.s){
    case STATE_HEADER_1:
      if(HEADER_1 == c){
        fsm.have_rx_len = 0;
        fsm.frame[fsm.have_rx_len++] = c;
        fsm.s = STATE_HEADER_2;
      }
    break;

    case STATE_HEADER_2:
      if(HEADER_2 == c){
        fsm.frame[fsm.have_rx_len++] = c;
        fsm.s = STATE_LEN1;
      }
      else {
        fsm.s = STATE_HEADER_1;
      }
    break;

    case STATE_LEN1:
      fsm.len = c;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.s = STATE_LEN2;
    break;

    case STATE_LEN2:
      fsm.len |= c<<8;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.payload_len = fsm.len - PAYLOAD_ADDITION_LEN;
      fsm.s = STATE_VER;
    break;

    case STATE_VER:
      if (VERSION == c) {
        fsm.ver = c;
        fsm.frame[fsm.have_rx_len++] = c;
        fsm.s = STATE_RECEIVER;
      }
      else {
        fsm.s = STATE_HEADER_1;
      }
    break;

    case STATE_RECEIVER:
      fsm.receiver = c;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.s = STATE_CRC;
    break;

    case STATE_CRC:
      if (c == calc_crc8(fsm.frame, fsm.have_rx_len)){
        fsm.header_crc8 = c;
        fsm.frame[fsm.have_rx_len++] = c;
        fsm.s = STATE_SENDER;
      }
      else {
        fsm.s = STATE_HEADER_1;
      }
    break;

    case STATE_SENDER:
      fsm.sender = c;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.s = STATE_ATTR;
    break;

    case STATE_ATTR:
      fsm.attr = c;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.s = STATE_ATTR;
    break;

    case STATE_DATA:
      if (fsm.have_rx_len < FRAME_MAX_SIZE) {
        fsm.frame[fsm.have_rx_len++] = c;
        if ((fsm.have_rx_len - HEADER_LEN) == fsm.len ) {
          fsm.s = STATE_CHECKSUM1;
        }
      }
      else {
        fsm.s = STATE_HEADER_1;
      }
    break;

    case STATE_CHECKSUM1:
      fsm.checksum = c;
      fsm.frame[fsm.have_rx_len++] = c;
      fsm.s = STATE_CHECKSUM2;
    break;

    case STATE_CHECKSUM2:
      fsm.checksum |= c<<8;
      if (fsm.checksum == 
          calculate_checksum(&fsm.frame[HEADER_LEN], fsm.have_rx_len - HEADER_LEN))
      { 
        fsm.frame[fsm.have_rx_len++] = c;
        ret = true;
      }
      fsm.s = STATE_HEADER_1;
    break;

    default:
      fsm.s = STATE_HEADER_1;
    break;
  }

  return ret;
}


void protocol_build_pack(scap_msg_t &msg, uint8_t *frame_buf, uint8_t &out_frame_len)
{
  uint32_t index = 0;

  if (out_frame_len < (FRAME_MIN_LEN + msg.payload_len)) {
    out_frame_len = 0;
    return;
  }

  frame_buf[index++] = HEADER_1;
  frame_buf[index++] = HEADER_2;
  uint16_t len = msg.payload_len + PAYLOAD_ADDITION_LEN;
  frame_buf[index++] = len & 0xFF;
  frame_buf[index++] = (len>>8) & 0xFF;
  frame_buf[index++] = VERSION;
  frame_buf[index++] = msg.peer;
  frame_buf[index++] = calc_crc8(frame_buf, HEADER_LEN);
  frame_buf[index++] = msg.sender;
  frame_buf[index++] = msg.attr;
  frame_buf[index++] = msg.seq;
  memcpy(frame_buf+index, msg.payload, msg.payload_len);
  index += msg.payload_len;
  uint16_t checksum = calculate_checksum(frame_buf + HEADER_LEN, index - HEADER_LEN);
  frame_buf[index++] = checksum & 0xFF;
  frame_buf[index++] = (checksum>>8) & 0xFF;
  out_frame_len = index;
}


/********************************************************************************/
// LOCAL FUN DEF
/********************************************************************************/
uint8_t calc_crc8(uint8_t *data, uint16_t length) {
  uint8_t i;
  uint8_t crc = 0x00;

  while(length--) {
      crc ^= *data++;
      for (i = 8; i > 0; --i) {
          if (crc & 0x80)
              crc = (crc << 1) ^ 0x07;
          else
              crc = (crc << 1);
      }
  }

  return crc;
}

uint16_t calculate_checksum(uint8_t *buffer, uint16_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (int j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}
