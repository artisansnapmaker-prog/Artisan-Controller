#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fms '
__author__ = '747'

from enum import Enum

class packet_type(Enum):
  SM2_CTRL_FW = 0x0001
  A400_CTRL_FW = 0x0002
  J1_CTRL_FW = 0x0003
  SM2_MODULE_FW = 0x0004
  ESP32_MODULE_FW = 0x0005

class ugr_ctrl_flag(Enum):
  UGR_NORMAL = 0x00
  UGR_FORCE = 0x01

class ugr_status(Enum):
  UGR_STATUS_FIRST_BURN = 0xAA00
  UGR_STATUS_FIRST_WAIT = 0xAA01
  UGR_STATUS_FIRST_START = 0xAA02
  UGR_STATUS_FIRST_TRANS = 0xAA03
  UGR_STATUS_FIRST_END = 0xAA04
  UGR_STATUS_FIRST_JUMP_APP = 0xAA05

class packet:
  def __init__(self):
    self.magic_string = "snapmaker update.bin"
    self.protocol_ver = 0x01
    self.pack_type = None
    self.ugr_ctrl_flag = ugr_ctrl_flag.UGR_NORMAL
    self.start_index = 0
    self.end_index = 0
    self.fw_version = None
    self.timestamp = None
    self.ugr_status = None
    self.fw_lenght = 0
    self.fw_checksum = 0
    self.fw_runaddr = 0
    self.channel = 0
    self.peer = 0
    self.packet_checksum = 0

  def pack(self):
    payload = bytearray(0)
    payload.extend(bytes(self.magic_string, encoding="utf-8"))
    # frame_str = " ".join(["{:02x}".format(x) for x in payload])
    # print(frame_str)
    payload.append(self.protocol_ver & 0xFF)
    payload.append((self.pack_type>>8) & 0xFF)
    payload.append(self.pack_type & 0xFF)
    payload.append(self.ugr_ctrl_flag & 0xFF)
    payload.append((self.start_index>>8) & 0xFF)
    payload.append(self.start_index & 0xFF)
    payload.append((self.end_index>>8) & 0xFF)
    payload.append(self.end_index & 0xFF)
    
    b = bytearray(self.fw_version, encoding="utf-8")
    l = len(b)
    for i in range(l, 32):
      b.append(0)
    payload.extend(b)

    b = bytearray(self.timestamp, encoding="utf-8")
    l = len(b)
    for i in range(l, 20):
      b.append(0)
    payload.extend(b)

    payload.append((self.ugr_status>>8) & 0xFF)
    payload.append(self.ugr_status & 0xFF)

    payload.append((self.fw_lenght>>24) & 0xFF)
    payload.append((self.fw_lenght>>16) & 0xFF)
    payload.append((self.fw_lenght>>8) & 0xFF)
    payload.append(self.fw_lenght & 0xFF)

    payload.append((self.fw_checksum>>24) & 0xFF)
    payload.append((self.fw_checksum>>16) & 0xFF)
    payload.append((self.fw_checksum>>8) & 0xFF)
    payload.append(self.fw_checksum & 0xFF)
    
  
if __name__=='__main__':
    pass