#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fms '
__author__ = '747'

from enum import Enum

PROTOCOL_VER = 0x01
PROTOCOL_MASK = 0x03

def crc8(data):
  crc = 0
  for c in data:
    crc ^= c
    for i in range(8,0,-1):
      if (crc & 0x80):
        crc = (crc << 1) ^ 0x07
      else:
        crc = (crc << 1)

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
    self.HEADER_1 = 0xFF
    self.HEADER_2 = 0x55
    self.state = fsm_state.STATE_HEADER_1
    self.exp_len = 0
    self.have_rx_len = 0
    self.checksum = 0
    self.receiver = -1
    self.sender = -1
    self.seq = -1
    self.frame = bytearray(0)

  def push_char(self, c):
    ret = False
    if self.state == fsm_state.STATE_HEADER_1:
      if self.HEADER_1 == c:
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
      self.exp_len += c 
      self.state = fsm_state.STATE_LEN2

    elif self.state == fsm_state.STATE_LEN2:
      self.exp_len += (c<<8) 
      self.state = fsm_state.STATE_VER

    elif self.state == fsm_state.STATE_VER:
      if PROTOCOL_VER != c:
        self.state = fsm_state.STATE_HEADER_1
      else:
        self.state = fsm_state.STATE_RECEIVER

    elif self.state == fsm_state.STATE_RECEIVER:
      self.receiver = c
      self.state = fsm_state.STATE_CRC

    elif self.state == fsm_state.STATE_CRC:
      data = self.frame[:]
      if c == crc8(data):
        self.stjate = fsm_state.STATE_SENDER
      else:
        self.state = fsm_state.STATE_HEADER_1

    elif self.state == fsm_state.STATE_SENDER:
      self.sender = c
      self.state = fsm_state.STATE_ATTR

    elif self.state == fsm_state.STATE_ATTR:
      self.attr = c
      self.state = fsm_state.STATE_SEQ1

    elif self.state == fsm_state.STATE_SEQ1:
      self.seq = c
      self.state = fsm_state.STATE_SEQ2

    elif self.state == fsm_state.STATE_SEQ2:
      self.seq += (c<<8)
      self.state = fsm_state.STATE_DATA

    elif self.state == fsm_state.STATE_SEQ2:
      self.seq += (c<<8)
      self.state = fsm_state.STATE_DATA

    elif self.state == fsm_state.STATE_LEN:
      self.exp_len = c
      self.have_rx_len = 0
      self.frame.append(c)
      if self.exp_len > 0:
        self.state = fsm_state.STATE_DATA
      else:
        self.state = fsm_state.STATE_CHECKSUM
    elif self.state == fsm_state.STATE_DATA:
      self.frame.append(c)
      self.have_rx_len += 1
      if self.have_rx_len == self.exp_len:
        self.state = fsm_state.STATE_CHECKSUM
    elif self.state == fsm_state.STATE_CHECKSUM:
      self.frame.append(c)
      for cc in self.frame:
          self.checksum ^= cc
      if 0 == self.checksum:
        ret = True
      self.state = fsm_state.STATE_HEADER_1
    return ret

  def build_frame(self, pl):
    checksum = 0
    frame = bytearray(0)
    frame.append(self.HEADER_1)
    frame.append(self.HEADER_2)
    frame.append(PROTOCOL_VER & PROTOCOL_MASK)
    frame.append(len(pl))
    frame.extend(pl)
    for c in frame:
      checksum ^= c
    frame.append(checksum)
    return frame

  def pars_ret_frame(self, ret_frame):
    cmd = ret_frame[4]
    seq = ret_frame[5]
    ret = ret_frame[6]
    return cmd, seq, ret 
    
  
if __name__=='__main__':
    pass