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
#ifndef SNAPMAKER_CAN_CHANNEL_H_
#define SNAPMAKER_CAN_CHANNEL_H_

#include "MapleFreeRTOS1030.h"

#include "../common/error.h"
#include "../common/ring_buffer.h"

#define CAN_MAC_QUEUE_SIZE        16

#define CAN_STD_CMD_QUEUE_SIZE    10
#define CAN_STD_CMD_ELEMENT_SIZE  10

#define CAN_EXT_CMD_QUEUE_SIZE    1024


enum CANBaudrate: uint8_t {
  CAN_BUADRATE_125K,
  CAN_BUADRATE_250K,
  CAN_BUADRATE_500K,
  CAN_BUADRATE_1M
};

typedef struct CANBaudrateSet {
  uint32_t sjw;
  uint32_t bs1;
  uint32_t bs2;
  uint32_t prescale;
} CANBaudrateSet_t;


enum CanFrameType {
  CAN_FRAME_STD,
  CAN_FRAME_EXT,

  CAN_FRAME_STD_DATA,
  CAN_FRAME_EXT_DATA,
  CAN_FRAME_STD_REMOTE,
  CAN_FRAME_EXT_REMOTE,

  CAN_FRAME_INVALID
};


enum CanChannelNumber {
  CAN_CH_1,
  CAN_CH_2,

  CAN_CH_MAX
};

typedef struct CanPacket {
  CanChannelNumber  ch;
  CanFrameType      ft;
  uint32_t          id;

  uint16_t          length;
  uint8_t           *data;
} CanPacket_t;


typedef struct CanStdDataFrame {
  union {
    struct {
      uint16_t  msg_id:   9;  // message id
      uint16_t  reserved: 1;  // reserved, main controller fill 0, module fill 1
      uint16_t  type:     1;  // indicates type of this frame, 0 = request, 1 = ack
      uint16_t  length:   5;  // indicates length of data field: 1 - 8
    } bits;

    uint16_t val;
  } id;               // ID field of CAN standard data frame

  uint8_t   data[8];  // data filed of CAN standard data frame
} CanStdDataFrame_t;


typedef bool (*CANIrqCallback_t)(CanStdDataFrame_t &cmd);

class CanChannel {
  public:
    err_code_t Init(CANIrqCallback_t irq_cb);

    err_code_t Write(CanPacket_t &packet);

    int32_t Read(CanFrameType ft, uint8_t *pdu, int32_t l);

    int32_t Available(CanFrameType ft);

    void Irq(CanChannelNumber ch, uint8_t fifo_index);

    RingBuffer<uint8_t> &ext_cmd() { return ext_cmd_; }

  private:
    err_code_t hal_init(void);
    err_code_t config_baudrate(CanChannelNumber bus, CANBaudrateSet_t br);
    err_code_t config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num);
    uint32_t canbus_send_packet(uint32_t id, uint8_t port_num, CanFrameType frame_type, uint8_t data_len, uint8_t *p_data);
    uint8_t canbus_parse_data(uint32_t *id, uint8_t *id_type, uint8_t port_num, uint8_t *frame_type, uint8_t *p_data, uint8_t *len, uint8_t fifo_num);

  private:
    RingBuffer<uint32_t> mac_id_;
    RingBuffer<uint8_t> ext_cmd_;

    CanStdDataFrame_t std_cmd_[CAN_STD_CMD_QUEUE_SIZE];
    uint8_t std_cmd_r_;
    uint8_t std_cmd_w_;
    uint8_t std_cmd_in_q_;

    CANIrqCallback_t irq_cb_;

    SemaphoreHandle_t lock_[CAN_CH_MAX];
    SemaphoreHandle_t can_lock;
};

extern CanChannel can;

#endif  // #define CAN_CHANNEL_H_
