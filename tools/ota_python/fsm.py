#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fms '
__author__ = '747'

import argparse
from enum import Enum


PROTOCOL_VER = 0x01
PROTOCOL_MASK = 0x03
RX_ID = 0x02
TX_ID = 0x01


def crc8(data):
  crc = 0
  for c in data:
    crc ^= c
    for i in range(8,0,-1):
      if (crc & 0x80):
        crc = (crc << 1) ^ 0x07
      else:
        crc = (crc << 1)
  return crc & 0xFF;

def crc16(data):
  checksum = 0
  l = len(data)

  for j in range(0, (int)(l / 2) * 2, 2):
    checksum += ((data[j]<<8) | data[j+1])

  if (l % 2):
    checksum += data[l - 1]

  while checksum > 0xFFFF:
    checksum = ((checksum >> 16) & 0xFFFF) + (checksum & 0xFFFF)

  checksum = checksum & 0xFFFF;
  checksum = ~checksum
  return checksum

class fsm_state(Enum):
  STATE_HEADER_1 = 0
  STATE_HEADER_2 = 1
  STATE_LEN1 = 2
  STATE_LEN2 = 3
  STATE_VER = 4
  STATE_RECEIVER = 5
  STATE_CRC = 6
  STATE_SENDER = 7
  STATE_ATTR = 8
  STATE_SEQ1 = 9
  STATE_SEQ2 = 10
  STATE_DATA = 11
  STATE_CHECKSUM1 = 12
  STATE_CHECKSUM2 = 13

class fsm:
  def __init__(self):
    self.HEADER_1 = 0xAA
    self.HEADER_2 = 0x55
    self.ATTR_REQ = 0
    self.ATTR_ACK = 1
    self.state = fsm_state.STATE_HEADER_1
    self.exp_len = 0
    self.have_rx_len = 0
    self.rx_checksum = 0
    self.cacl_checksum = 0
    self.receiver = -1
    self.sender = -1
    self.seq = -1
    self.frame = bytearray(0)

  def push_char(self, c):
    ret = False
    if self.state == fsm_state.STATE_HEADER_1:
      if self.HEADER_1 == c:
        self.have_rx_len = 0
        self.frame.clear()
        self.frame.append(c)
        self.state = fsm_state.STATE_HEADER_2

    elif self.state == fsm_state.STATE_HEADER_2:
      if self.HEADER_2 == c:
        self.frame.append(c)
        self.state = fsm_state.STATE_LEN1
      else:
        self.state = fsm_state.STATE_HEADER_1

    elif self.state == fsm_state.STATE_LEN1:
      self.exp_len = c 
      self.frame.append(c)
      self.state = fsm_state.STATE_LEN2

    elif self.state == fsm_state.STATE_LEN2:
      self.exp_len += (c<<8)
      self.exp_len -= 6
      self.frame.append(c)
      self.state = fsm_state.STATE_VER

    elif self.state == fsm_state.STATE_VER:
      if PROTOCOL_VER != c:
        self.state = fsm_state.STATE_HEADER_1
      else:
        self.frame.append(c)
        self.state = fsm_state.STATE_RECEIVER

    elif self.state == fsm_state.STATE_RECEIVER:
      self.receiver = c
      self.frame.append(c)
      self.state = fsm_state.STATE_CRC

    elif self.state == fsm_state.STATE_CRC:
      data = self.frame[:]
      if c == crc8(data):
        self.frame.append(c)
        self.state = fsm_state.STATE_SENDER
      else:
        self.state = fsm_state.STATE_HEADER_1

    elif self.state == fsm_state.STATE_SENDER:
      self.sender = c
      self.frame.append(c)
      self.state = fsm_state.STATE_ATTR

    elif self.state == fsm_state.STATE_ATTR:
      self.attr = c
      self.frame.append(c)
      self.state = fsm_state.STATE_SEQ1

    elif self.state == fsm_state.STATE_SEQ1:
      self.seq = c
      self.frame.append(c)
      self.state = fsm_state.STATE_SEQ2

    elif self.state == fsm_state.STATE_SEQ2:
      self.seq += (c<<8)
      self.frame.append(c)
      self.state = fsm_state.STATE_DATA
        
    elif self.state == fsm_state.STATE_DATA:
      self.frame.append(c)
      self.have_rx_len += 1
      if self.have_rx_len == self.exp_len:
        self.cacl_checksum = crc16(self.frame[7:]) & 0xFFFF
        self.state = fsm_state.STATE_CHECKSUM1
        
    elif self.state == fsm_state.STATE_CHECKSUM1:
      self.rx_checksum = c
      self.frame.append(c)
      self.state = fsm_state.STATE_CHECKSUM2

    elif self.state == fsm_state.STATE_CHECKSUM2:
      self.rx_checksum += (c<<8)
      self.frame.append(c)
      if self.cacl_checksum == self.rx_checksum:
        ret = True
      self.state = fsm_state.STATE_HEADER_1
      
    return ret

  def build_frame(self, pl, peer, attr = 0, seq = 0):
    lenght = len(pl) + 6
    data_checksum = 0
    frame = bytearray(0)
    frame.append(self.HEADER_1)
    frame.append(self.HEADER_2)
    frame.append(lenght & 0xff)
    frame.append((lenght>>8) & 0xff)
    frame.append(PROTOCOL_VER)
    frame.append(peer)
    head_crc = crc8(frame)
    frame.append(head_crc)
    frame.append(TX_ID)
    frame.append(attr)
    frame.append(seq & 0xff)
    frame.append((seq>>8) & 0xff)
    frame.extend(pl)
    data_checksum = crc16(frame[7:])
    frame.append(data_checksum & 0xff)
    frame.append((data_checksum>>8) & 0xff)
    return frame

  def pars_ret_frame(self, ret_frame):
    cmd_set = ret_frame[11]
    cmd_id = ret_frame[12]
    ret = ret_frame[13]
    return cmd_set, cmd_id, ret
    
  
if __name__=='__main__':
    pass